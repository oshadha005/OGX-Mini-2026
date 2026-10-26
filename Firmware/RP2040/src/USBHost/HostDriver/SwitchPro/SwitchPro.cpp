#include <cstring>
#include <array>
#include <tuple>

#include "host/usbh.h"
#include "class/hid/hid_host.h"
#include "common/tusb_types.h"

#include "Board/Config.h"
#include "Board/board_api.h"
#include "Board/ogxm_log.h"
#if defined(CONFIG_EN_USB_HOST)
#include "pio_usb.h"
#endif

#include "USBHost/HostDriver/SwitchPro/SwitchPro.h"
#include "USBHost/HostDriver/SwitchPro/Switch2UsbInitPackets.h"
#include "USBHost/HostManager.h"
#include "Input/InputSlot.h"
#include "Gamepad/MotionImu.h"

namespace {

#if defined(CONFIG_EN_USB_HOST)
void service_usb_host() {
    pio_usb_host_frame();
    tuh_task();
}
#else
void service_usb_host() {
    tuh_task();
}
#endif

bool try_hid_out_id(uint8_t address, uint8_t instance, uint8_t report_id, const void* report, uint16_t len) {
    if (!tuh_hid_send_ready(address, instance)) {
        return false;
    }
    return tuh_hid_send_report(address, instance, report_id, report, len);
}

bool try_hid_out(uint8_t address, uint8_t instance, const void* report, uint16_t len) {
    return try_hid_out_id(address, instance, 0, report, len);
}

bool switch_rearm_hid_in(uint8_t address, uint8_t instance) {
    const bool ok = tuh_hid_receive_report(address, instance);
    InputSlot::note_rearm(ok);
    return ok;
}

constexpr uint8_t kWiredHandshakeMaxRetries = 6;
constexpr uint8_t kSubcmdMaxRetries = 6;
constexpr uint32_t kSubcmdSpacingMs = 50U;
constexpr uint32_t kZeroImuWakeIntervalMs = 1000U;
constexpr uint16_t kZeroImuWakeThreshold = 80U;
constexpr uint32_t kActiveRumbleIntervalMs = 32U;
constexpr uint32_t kIdleKeepaliveIntervalMs = 1000U;

struct SlotRuntimeState {
    uint32_t last_cmd_ms{0};
    uint32_t last_rumble_ms{0};
    uint16_t zero_imu_frames{0};
    uint8_t last_rumble_l{0};
    uint8_t last_rumble_r{0};
    bool wake_send_mode_next{false};
};

std::array<SlotRuntimeState, MAX_GAMEPADS> s_slot_runtime{};

SlotRuntimeState& get_slot_runtime(uint8_t idx) {
    const size_t i = (static_cast<size_t>(idx) < s_slot_runtime.size()) ? static_cast<size_t>(idx) : 0U;
    return s_slot_runtime[i];
}

} // namespace

bool SwitchProHost::is_switch2_usb_family(uint16_t vid, uint16_t pid)
{
    if (vid != 0x057Eu)
    {
        return false;
    }
    switch (pid)
    {
    case 0x2066u:
    case 0x2067u:
    case 0x2069u:
    case 0x2073u:
        return true;
    default:
        return false;
    }
}

SwitchProHost::~SwitchProHost()
{
    switch2_cancel_bringup();
}

void SwitchProHost::disconnect_cb(Gamepad& gamepad, uint8_t address, uint8_t instance)
{
    (void)gamepad;
    (void)address;
    (void)instance;
    get_slot_runtime(idx_) = SlotRuntimeState{};
    switch2_cancel_bringup();
}

uintptr_t SwitchProHost::switch2_make_cb_token(uint8_t daddr, uint8_t instance)
{
    return (static_cast<uintptr_t>(instance) << 8) | daddr;
}

SwitchProHost* SwitchProHost::switch2_lookup(uint8_t daddr, uint8_t instance)
{
    return HostManager::get_instance().get_switch_pro_host(daddr, instance);
}

void SwitchProHost::switch2_cancel_bringup()
{
    switch2_bringup_active_ = false;
    switch2_bringup_pending_retry_ = false;
    switch2_next_packet_ms_ = 0;
    switch2_pkt_idx_ = 0;
    switch2_bulk_keepalive_busy_ = false;

    if (switch2_dev_addr_ != 0)
    {
        if (switch2_ep_out_ != 0)
        {
            (void)tuh_edpt_abort_xfer(switch2_dev_addr_, switch2_ep_out_);
        }
        if (switch2_ep_in_ != 0)
        {
            (void)tuh_edpt_abort_xfer(switch2_dev_addr_, switch2_ep_in_);
        }
    }

    switch2_ep_out_ = 0;
    switch2_ep_in_ = 0;
}

void SwitchProHost::initialize(Gamepad& gamepad, uint8_t address, uint8_t instance,
                              const uint8_t* report_desc, uint16_t desc_len)
{
    (void)report_desc;
    (void)desc_len;

    std::memset(&out_report_, 0, sizeof(out_report_));
    std::memset(&prev_in_report_, 0, sizeof(prev_in_report_));
    std::memset(prev_imu_, 0, sizeof(prev_imu_));
    get_slot_runtime(idx_) = SlotRuntimeState{};

    sequence_counter_ = 0;
    hid_instance_ = instance;
    switch2_dev_addr_ = address;
    switch2_gamepad_ = &gamepad;
    init_state_ = InitState::USB_MAC;
    pending_usb_ack_ = 0;
    pending_subcmd_ack_ = 0;
    init_step_retries_ = 0;
    usb_timeout_sent_ = false;
    set_mode_acked_ = false;
    switch2_cancel_bringup();

    uint16_t vid = 0;
    uint16_t pid = 0;
    tuh_vid_pid_get(address, &vid, &pid);

    if (is_switch2_usb_family(vid, pid))
    {
        switch2_start_bringup();
    }
    else
    {
        start_usb_wired_init(address, instance);
    }
}

void SwitchProHost::switch2_start_bringup()
{
    switch2_bringup_active_ = true;
    switch2_pkt_idx_ = 0;
    switch2_ep_out_ = 0;
    switch2_ep_in_ = 0;
    switch2_next_packet_ms_ = 0;
    switch2_start_config_read();
}

void SwitchProHost::switch2_start_config_read()
{
    if (!tuh_descriptor_get_configuration(
            switch2_dev_addr_,
            0,
            switch2_cfg_buf_.data(),
            static_cast<uint16_t>(switch2_cfg_buf_.size()),
            switch2_cfg_cb,
            switch2_make_cb_token(switch2_dev_addr_, hid_instance_)))
    {
        switch2_finish_bringup();
    }
}

bool SwitchProHost::switch2_parse_open_iface1(const uint8_t* desc_cfg, uint16_t buflen)
{
    switch2_ep_out_ = 0;
    switch2_ep_in_ = 0;

    if (buflen < sizeof(tusb_desc_configuration_t))
    {
        return false;
    }

    auto const* cfg = reinterpret_cast<tusb_desc_configuration_t const*>(desc_cfg);
    uint16_t total = tu_le16toh(cfg->wTotalLength);
    uint16_t len = TU_MIN(total, buflen);
    uint8_t const* end = desc_cfg + len;
    uint8_t const* p = tu_desc_next(desc_cfg);

    while (p < end)
    {
        if (tu_desc_type(p) != TUSB_DESC_INTERFACE)
        {
            p = tu_desc_next(p);
            continue;
        }

        auto const* itf = reinterpret_cast<tusb_desc_interface_t const*>(p);
        if (itf->bInterfaceNumber != 1 || itf->bAlternateSetting != 0)
        {
            p = tu_desc_next(p);
            continue;
        }

        p = tu_desc_next(p);
        int endpoint = 0;
        while (endpoint < itf->bNumEndpoints && p < end)
        {
            if (tu_desc_type(p) != TUSB_DESC_ENDPOINT)
            {
                p = tu_desc_next(p);
                continue;
            }

            auto const* ep = reinterpret_cast<tusb_desc_endpoint_t const*>(p);
            if (ep->bmAttributes.xfer == TUSB_XFER_BULK)
            {
                if (!tuh_edpt_open(switch2_dev_addr_, ep))
                {
                    return false;
                }
                if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_OUT && switch2_ep_out_ == 0)
                {
                    switch2_ep_out_ = ep->bEndpointAddress;
                }
                else if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN && switch2_ep_in_ == 0)
                {
                    switch2_ep_in_ = ep->bEndpointAddress;
                }
            }
            endpoint++;
            p = tu_desc_next(p);
        }

        return switch2_ep_out_ != 0;
    }

    return false;
}

void SwitchProHost::switch2_cfg_cb(tuh_xfer_s* xfer)
{
    const uint8_t daddr = xfer->daddr;
    const uint8_t instance = static_cast<uint8_t>(xfer->user_data >> 8);
    SwitchProHost* self = switch2_lookup(daddr, instance);
    if (self == nullptr || !self->switch2_bringup_active_)
    {
        return;
    }

    if (xfer->result != XFER_RESULT_SUCCESS ||
        xfer->actual_len < sizeof(tusb_desc_configuration_t))
    {
        self->switch2_finish_bringup();
        return;
    }

    uint16_t actual = static_cast<uint16_t>(xfer->actual_len);
    auto const* cfg = reinterpret_cast<tusb_desc_configuration_t const*>(self->switch2_cfg_buf_.data());
    const uint16_t total = tu_le16toh(cfg->wTotalLength);
    if (total > actual && total <= self->switch2_cfg_buf_.size())
    {
        if (!tuh_descriptor_get_configuration(
                self->switch2_dev_addr_,
                0,
                self->switch2_cfg_buf_.data(),
                total,
                switch2_cfg_cb,
                switch2_make_cb_token(self->switch2_dev_addr_, self->hid_instance_)))
        {
            self->switch2_finish_bringup();
        }
        return;
    }

    if (!self->switch2_parse_open_iface1(self->switch2_cfg_buf_.data(), actual))
    {
        self->switch2_finish_bringup();
        return;
    }

    self->switch2_pkt_idx_ = 0;
    self->switch2_submit_out_packet();
}

void SwitchProHost::switch2_out_cb(tuh_xfer_s* xfer)
{
    const uint8_t daddr = xfer->daddr;
    const uint8_t instance = static_cast<uint8_t>(xfer->user_data >> 8);
    SwitchProHost* self = switch2_lookup(daddr, instance);
    if (self == nullptr || !self->switch2_bringup_active_)
    {
        return;
    }

    if (xfer->result != XFER_RESULT_SUCCESS)
    {
        self->switch2_finish_bringup();
        return;
    }

    if (self->switch2_ep_in_ != 0)
    {
        self->switch2_submit_in_read();
    }
    else
    {
        self->switch2_after_packet_round();
    }
}

void SwitchProHost::switch2_in_cb(tuh_xfer_s* xfer)
{
    const uint8_t daddr = xfer->daddr;
    const uint8_t instance = static_cast<uint8_t>(xfer->user_data >> 8);
    SwitchProHost* self = switch2_lookup(daddr, instance);
    if (self == nullptr || !self->switch2_bringup_active_)
    {
        return;
    }
    (void)xfer;
    self->switch2_after_packet_round();
}

void SwitchProHost::switch2_keepalive_cb(tuh_xfer_s* xfer)
{
    const uint8_t daddr = xfer->daddr;
    const uint8_t instance = static_cast<uint8_t>(xfer->user_data >> 8);
    SwitchProHost* self = switch2_lookup(daddr, instance);
    if (self != nullptr)
    {
        self->switch2_bulk_keepalive_busy_ = false;
    }
    (void)daddr;
    (void)instance;
}

void SwitchProHost::switch2_send_bulk_keepalive()
{
    if (switch2_ep_out_ == 0 || switch2_bulk_keepalive_busy_)
    {
        return;
    }

    static const uint8_t kKeepalive[] = {0x03, 0x91, 0x00, 0x01, 0x00, 0x00, 0x00};
    if (sizeof(kKeepalive) > switch2_bulk_out_buf_.size())
    {
        return;
    }

    std::memcpy(switch2_bulk_out_buf_.data(), kKeepalive, sizeof(kKeepalive));

    tuh_xfer_t xfer{};
    xfer.daddr = switch2_dev_addr_;
    xfer.ep_addr = switch2_ep_out_;
    xfer.buflen = static_cast<uint16_t>(sizeof(kKeepalive));
    xfer.buffer = switch2_bulk_out_buf_.data();
    xfer.complete_cb = switch2_keepalive_cb;
    xfer.user_data = switch2_make_cb_token(switch2_dev_addr_, hid_instance_);

    switch2_bulk_keepalive_busy_ = true;
    if (!tuh_edpt_xfer(&xfer))
    {
        switch2_bulk_keepalive_busy_ = false;
    }
}

void SwitchProHost::switch2_submit_out_packet()
{
    const Switch2UsbBulkPacket& pkt = SWITCH2_USB_BULK_PACKETS[switch2_pkt_idx_];
    if (pkt.len > switch2_bulk_out_buf_.size())
    {
        switch2_finish_bringup();
        return;
    }

    std::memcpy(switch2_bulk_out_buf_.data(), pkt.data, pkt.len);

    tuh_xfer_t xfer{};
    xfer.daddr = switch2_dev_addr_;
    xfer.ep_addr = switch2_ep_out_;
    xfer.buflen = pkt.len;
    xfer.buffer = switch2_bulk_out_buf_.data();
    xfer.complete_cb = switch2_out_cb;
    xfer.user_data = switch2_make_cb_token(switch2_dev_addr_, hid_instance_);

    if (!tuh_edpt_xfer(&xfer))
    {
        switch2_next_packet_ms_ = board_api::ms_since_boot() + 12U;
    }
}

void SwitchProHost::switch2_submit_in_read()
{
    tuh_xfer_t xfer{};
    xfer.daddr = switch2_dev_addr_;
    xfer.ep_addr = switch2_ep_in_;
    xfer.buflen = 64;
    xfer.buffer = switch2_bulk_in_buf_.data();
    xfer.complete_cb = switch2_in_cb;
    xfer.user_data = switch2_make_cb_token(switch2_dev_addr_, hid_instance_);

    if (!tuh_edpt_xfer(&xfer))
    {
        switch2_next_packet_ms_ = board_api::ms_since_boot() + 12U;
    }
}

void SwitchProHost::switch2_after_packet_round()
{
    switch2_pkt_idx_++;
    if (switch2_pkt_idx_ >= SWITCH2_USB_BULK_PACKET_COUNT)
    {
        switch2_finish_bringup();
        return;
    }
    switch2_next_packet_ms_ = board_api::ms_since_boot() + 12U;
}

void SwitchProHost::switch2_poll_bringup()
{
    if (!switch2_bringup_active_ || switch2_next_packet_ms_ == 0)
    {
        return;
    }
    if (board_api::ms_since_boot() < switch2_next_packet_ms_)
    {
        return;
    }
    switch2_next_packet_ms_ = 0;
    switch2_submit_out_packet();
}

void SwitchProHost::switch2_finish_bringup()
{
    const bool bulk_complete = switch2_pkt_idx_ >= SWITCH2_USB_BULK_PACKET_COUNT;
    switch2_bringup_active_ = false;
    switch2_next_packet_ms_ = 0;

    if (switch2_gamepad_ == nullptr)
    {
        return;
    }

    if (bulk_complete)
    {
        init_state_ = InitState::DONE;
        pending_usb_ack_ = 0;
        pending_subcmd_ack_ = 0;
        send_usb_disable_timeout(switch2_dev_addr_, hid_instance_);
        switch2_last_bulk_keepalive_ms_ = board_api::ms_since_boot();
        switch2_send_bulk_keepalive();
        tuh_hid_receive_report(switch2_dev_addr_, hid_instance_);
        return;
    }

    switch2_bringup_pending_retry_ = true;
}

bool SwitchProHost::send_usb_wired_command(uint8_t address, uint8_t instance, uint8_t subcmd)
{
    uint8_t payload[63] = {0};
    payload[0] = subcmd;
    pending_usb_ack_ = subcmd;
    return try_hid_out_id(address, instance, SwitchPro::REPORT_ID_USB_OUT, payload, sizeof(payload));
}

void SwitchProHost::start_usb_wired_init(uint8_t address, uint8_t instance)
{
    SlotRuntimeState& slot = get_slot_runtime(idx_);
    init_state_ = InitState::USB_MAC;
    init_step_retries_ = 0;
    pending_usb_ack_ = SwitchPro::CMD::USB_SUB_MAC;
    if (send_usb_wired_command(address, instance, SwitchPro::CMD::USB_SUB_MAC))
    {
        slot.last_cmd_ms = board_api::ms_since_boot();
    }
    (void)tuh_hid_receive_report(address, instance);
}

void SwitchProHost::fill_neutral_rumble(SwitchPro::OutReport& out)
{
    out.rumble_l[0] = 0x00;
    out.rumble_l[1] = 0x01;
    out.rumble_l[2] = 0x40;
    out.rumble_l[3] = 0x40;
    out.rumble_r[0] = 0x00;
    out.rumble_r[1] = 0x01;
    out.rumble_r[2] = 0x40;
    out.rumble_r[3] = 0x40;
}

void SwitchProHost::advance_after_usb_ack(Gamepad& gamepad, uint8_t address, uint8_t instance)
{
    (void)gamepad;
    SlotRuntimeState& slot = get_slot_runtime(idx_);
    init_step_retries_ = 0;

    switch (init_state_)
    {
        case InitState::USB_MAC:
            init_state_ = InitState::USB_HANDSHAKE1;
            pending_usb_ack_ = SwitchPro::CMD::USB_SUB_HANDSHAKE;
            if (send_usb_wired_command(address, instance, SwitchPro::CMD::USB_SUB_HANDSHAKE))
            {
                slot.last_cmd_ms = board_api::ms_since_boot();
            }
            break;
        case InitState::USB_HANDSHAKE1:
            init_state_ = InitState::USB_BAUD;
            pending_usb_ack_ = SwitchPro::CMD::USB_SUB_BAUD;
            if (send_usb_wired_command(address, instance, SwitchPro::CMD::USB_SUB_BAUD))
            {
                slot.last_cmd_ms = board_api::ms_since_boot();
            }
            break;
        case InitState::USB_BAUD:
            init_state_ = InitState::USB_HANDSHAKE2;
            pending_usb_ack_ = SwitchPro::CMD::USB_SUB_HANDSHAKE;
            if (send_usb_wired_command(address, instance, SwitchPro::CMD::USB_SUB_HANDSHAKE))
            {
                slot.last_cmd_ms = board_api::ms_since_boot();
            }
            break;
        case InitState::USB_HANDSHAKE2:
            init_state_ = InitState::USB_NO_TIMEOUT;
            send_usb_disable_timeout_and_probe(address, instance);
            break;
        default:
            break;
    }
}

void SwitchProHost::send_usb_disable_timeout_and_probe(uint8_t address, uint8_t instance)
{
    (void)init_profile_;
    SlotRuntimeState& slot = get_slot_runtime(idx_);

    send_usb_disable_timeout(address, instance);
    usb_timeout_sent_ = true;
    pending_usb_ack_ = 0;
    init_step_retries_ = 0;
    slot.last_cmd_ms = board_api::ms_since_boot();

    /* Move cleanly to IMU enable on the next 50ms window after 0x80 0x04 completes. */
    init_state_ = InitState::IMU;
    pending_subcmd_ack_ = SwitchPro::CMD::GYRO;
}

bool SwitchProHost::send_subcmd_report(uint8_t address, uint8_t instance, uint8_t subcmd,
                                       const uint8_t* args, uint8_t args_len)
{
    std::memset(&out_report_, 0, sizeof(out_report_));
    fill_neutral_rumble(out_report_);
    out_report_.command = SwitchPro::REPORT_ID_OUTPUT_SUBCMD;
    out_report_.sequence_counter = get_output_sequence_counter();
    out_report_.sub_command = subcmd;
    if (args && args_len > 0) {
        const uint8_t n = (args_len > sizeof(out_report_.sub_command_args))
                              ? static_cast<uint8_t>(sizeof(out_report_.sub_command_args))
                              : args_len;
        std::memcpy(out_report_.sub_command_args, args, n);
    }
    pending_subcmd_ack_ = subcmd;
    const uint16_t len = static_cast<uint16_t>(11 + (args_len > 0 ? args_len : 0));
    return try_hid_out(address, instance, &out_report_, len > 12 ? len : 12);
}

void SwitchProHost::send_usb_disable_timeout(uint8_t address, uint8_t instance)
{
    uint8_t usb_out[63] = {0};
    usb_out[0] = SwitchPro::CMD::USB_SUB_DISABLE_TIMEOUT;
    (void)try_hid_out_id(address, instance, SwitchPro::REPORT_ID_USB_OUT, usb_out, sizeof(usb_out));
}

void SwitchProHost::init_switch_host(Gamepad& gamepad, uint8_t address, uint8_t instance)
{
    if (!tuh_hid_send_ready(address, instance))
    {
        return;
    }

    SlotRuntimeState& slot = get_slot_runtime(idx_);
    const uint32_t now = board_api::ms_since_boot();
    if (slot.last_cmd_ms != 0 && (now - slot.last_cmd_ms) < kSubcmdSpacingMs)
    {
        return;
    }

    if (init_state_ <= InitState::USB_HANDSHAKE2)
    {
        if (++init_step_retries_ >= kWiredHandshakeMaxRetries)
        {
            advance_after_usb_ack(gamepad, address, instance);
        }
        else if (pending_usb_ack_ != 0 && send_usb_wired_command(address, instance, pending_usb_ack_))
        {
            slot.last_cmd_ms = now;
        }
        return;
    }

    if (init_state_ == InitState::USB_NO_TIMEOUT)
    {
        send_usb_disable_timeout_and_probe(address, instance);
        return;
    }

    switch (init_state_)
    {
        case InitState::IMU:
        {
            if (init_step_retries_ >= kSubcmdMaxRetries)
            {
                init_state_ = InitState::FULL_REPORT;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                break;
            }
            const uint8_t enable = 1;
            if (send_subcmd_report(address, instance, SwitchPro::CMD::GYRO, &enable, 1))
            {
                ++init_step_retries_;
                slot.last_cmd_ms = now;
            }
            break;
        }
        case InitState::FULL_REPORT:
        {
            if (init_step_retries_ >= kSubcmdMaxRetries)
            {
                init_state_ = InitState::AWAIT_FULL_REPORT;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                break;
            }
            const uint8_t mode = SwitchPro::CMD::FULL_REPORT_MODE;
            if (send_subcmd_report(address, instance, SwitchPro::SUBCMD_SET_MODE, &mode, 1))
            {
                ++init_step_retries_;
                slot.last_cmd_ms = now;
            }
            break;
        }
        case InitState::AWAIT_FULL_REPORT:
        {
            /* Re-assert 0x40 0x01 after 0x03 0x30 for clones that reset IMU state on mode switch. */
            if (init_step_retries_ >= kSubcmdMaxRetries)
            {
                init_state_ = InitState::LED;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                break;
            }
            const uint8_t enable = 1;
            if (send_subcmd_report(address, instance, SwitchPro::CMD::GYRO, &enable, 1))
            {
                ++init_step_retries_;
                slot.last_cmd_ms = now;
            }
            break;
        }
        case InitState::LED:
        {
            if (init_step_retries_ >= 3)
            {
                init_state_ = InitState::DONE;
                pending_subcmd_ack_ = 0;
                init_step_retries_ = 0;
                break;
            }
            const uint8_t player_led = static_cast<uint8_t>(idx_ + 1);
            if (send_subcmd_report(address, instance, SwitchPro::CMD::LED, &player_led, 1))
            {
                ++init_step_retries_;
                slot.last_cmd_ms = now;
            }
            break;
        }
        default:
            break;
    }
}

uint8_t SwitchProHost::get_output_sequence_counter()
{
    uint8_t counter = sequence_counter_;
    sequence_counter_ = (sequence_counter_ + 1) & 0x0F;
    return counter;
}

static const uint8_t* switchpro_report_payload(const uint8_t* report, uint16_t len, uint16_t& out_payload_len)
{
    out_payload_len = len;
    if (len == sizeof(SwitchPro::InReport))
    {
        return report;
    }
    if (len >= 2U + sizeof(SwitchPro::InReport))
    {
        const uint8_t rid = report[0];
        if (rid == SwitchPro::REPORT_ID_STANDARD || rid == SwitchPro::REPORT_ID_SWITCH2_FULL ||
            rid == SwitchPro::REPORT_ID_FULL_ALT)
        {
            out_payload_len = static_cast<uint16_t>(len - 2U);
            return report + 2;
        }
    }
    return nullptr;
}

void SwitchProHost::apply_standard_input(Gamepad& gamepad, const uint8_t* report, uint16_t len)
{
    uint16_t payload_len = 0;
    const uint8_t* payload = switchpro_report_payload(report, len, payload_len);
    if (payload == nullptr || payload_len < sizeof(SwitchPro::InReport))
    {
        return;
    }

    constexpr uint16_t kMinImuPayloadLen = 11U + sizeof(MotionImu::SwitchImuSample);
    SlotRuntimeState& slot = get_slot_runtime(idx_);

    if (payload_len >= kMinImuPayloadLen)
    {
        const uint16_t imu_off = MotionImu::switch_usb_imu_offset(payload, payload_len);
        if (MotionImu::switch_imu_sample_is_zero(payload + imu_off))
        {
            if (slot.zero_imu_frames < 1000U)
            {
                ++slot.zero_imu_frames;
            }
        }
        else
        {
            slot.zero_imu_frames = 0;
        }
    }

    const SwitchPro::InReport* in_report = reinterpret_cast<const SwitchPro::InReport*>(payload);
    if (std::memcmp(&prev_in_report_.buttons, in_report->buttons, 9) == 0 &&
        MotionImu::switch_usb_imu_unchanged(prev_imu_, payload, payload_len))
    {
        return;
    }

    Gamepad::PadIn gp_in;

    if (in_report->buttons[0] & SwitchPro::Buttons0::Y)
        gp_in.buttons |= gamepad.MAP_BUTTON_X;
    if (in_report->buttons[0] & SwitchPro::Buttons0::B)
        gp_in.buttons |= gamepad.MAP_BUTTON_A;
    if (in_report->buttons[0] & SwitchPro::Buttons0::A)
        gp_in.buttons |= gamepad.MAP_BUTTON_B;
    if (in_report->buttons[0] & SwitchPro::Buttons0::X)
        gp_in.buttons |= gamepad.MAP_BUTTON_Y;
    if (in_report->buttons[2] & SwitchPro::Buttons2::L)
        gp_in.buttons |= gamepad.MAP_BUTTON_LB;
    if (in_report->buttons[0] & SwitchPro::Buttons0::R)
        gp_in.buttons |= gamepad.MAP_BUTTON_RB;
    if (in_report->buttons[1] & SwitchPro::Buttons1::L3)
        gp_in.buttons |= gamepad.MAP_BUTTON_R3;
    if (in_report->buttons[1] & SwitchPro::Buttons1::R3)
        gp_in.buttons |= gamepad.MAP_BUTTON_L3;
    if (in_report->buttons[1] & SwitchPro::Buttons1::MINUS)
        gp_in.buttons |= gamepad.MAP_BUTTON_BACK;
    if (in_report->buttons[1] & SwitchPro::Buttons1::PLUS)
        gp_in.buttons |= gamepad.MAP_BUTTON_START;
    if (in_report->buttons[1] & SwitchPro::Buttons1::HOME)
        gp_in.buttons |= gamepad.MAP_BUTTON_SYS;
    if (in_report->buttons[1] & SwitchPro::Buttons1::CAPTURE)
        gp_in.buttons |= gamepad.MAP_BUTTON_MISC;

    if (in_report->buttons[2] & SwitchPro::Buttons2::DPAD_UP)
        gp_in.dpad |= gamepad.MAP_DPAD_DOWN;
    if (in_report->buttons[2] & SwitchPro::Buttons2::DPAD_DOWN)
        gp_in.dpad |= gamepad.MAP_DPAD_UP;
    if (in_report->buttons[2] & SwitchPro::Buttons2::DPAD_LEFT)
        gp_in.dpad |= gamepad.MAP_DPAD_RIGHT;
    if (in_report->buttons[2] & SwitchPro::Buttons2::DPAD_RIGHT)
        gp_in.dpad |= gamepad.MAP_DPAD_LEFT;

    gp_in.trigger_l = in_report->buttons[2] & SwitchPro::Buttons2::ZL ? Range::MAX<uint8_t> : Range::MIN<uint8_t>;
    gp_in.trigger_r = in_report->buttons[0] & SwitchPro::Buttons0::ZR ? Range::MAX<uint8_t> : Range::MIN<uint8_t>;

    uint16_t joy_lx = in_report->joysticks[0] | ((in_report->joysticks[1] & 0xF) << 8);
    uint16_t joy_ly = (in_report->joysticks[1] >> 4) | (in_report->joysticks[2] << 4);
    uint16_t joy_rx = in_report->joysticks[3] | ((in_report->joysticks[4] & 0xF) << 8);
    uint16_t joy_ry = (in_report->joysticks[4] >> 4) | (in_report->joysticks[5] << 4);

    std::tie(gp_in.joystick_lx, gp_in.joystick_ly) =
        gamepad.scale_joystick_l(normalize_axis(joy_lx), normalize_axis(joy_ly), true);

    std::tie(gp_in.joystick_rx, gp_in.joystick_ry) =
        gamepad.scale_joystick_r(normalize_axis(joy_rx), normalize_axis(joy_ry), true);

    if (payload_len >= kMinImuPayloadLen) {
        gp_in.motion_source = Gamepad::PadIn::MOTION_SRC_SWITCH_USB;
        MotionImu::fill_from_switch_usb_payload(gp_in.accel, gp_in.gyro, payload, payload_len);
    }

    gamepad.set_pad_in_latest(gp_in);
    InputSlot::note_decoded();
    std::memcpy(&prev_in_report_, in_report, sizeof(SwitchPro::InReport));
    if (payload_len >= kMinImuPayloadLen) {
        const uint16_t imu_off = MotionImu::switch_usb_imu_offset(payload, payload_len);
        std::memcpy(prev_imu_, payload + imu_off, sizeof(prev_imu_));
    }
}

void SwitchProHost::process_report(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report,
                                   uint16_t len)
{
    if (switch2_bringup_active_)
    {
        (void)switch_rearm_hid_in(address, instance);
        return;
    }

    SlotRuntimeState& slot = get_slot_runtime(idx_);

    if (init_state_ != InitState::DONE)
    {
        if (len >= 2 && report[0] == SwitchPro::REPORT_ID_USB_INIT &&
            init_state_ <= InitState::USB_HANDSHAKE2)
        {
            const uint8_t ack = report[1];
            if (ack == pending_usb_ack_)
            {
                advance_after_usb_ack(gamepad, address, instance);
            }
        }
        else if (len >= 15 && report[0] == SwitchPro::REPORT_ID_SUBCMD)
        {
            const uint8_t reply_sub = report[14];
            if (init_state_ == InitState::IMU && reply_sub == SwitchPro::CMD::GYRO)
            {
                init_state_ = InitState::FULL_REPORT;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                init_switch_host(gamepad, address, instance);
            }
            else if (init_state_ == InitState::FULL_REPORT && reply_sub == SwitchPro::SUBCMD_SET_MODE)
            {
                set_mode_acked_ = true;
                init_state_ = InitState::AWAIT_FULL_REPORT;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                init_switch_host(gamepad, address, instance);
            }
            else if (init_state_ == InitState::AWAIT_FULL_REPORT && reply_sub == SwitchPro::CMD::GYRO)
            {
                init_state_ = InitState::LED;
                init_step_retries_ = 0;
                slot.last_cmd_ms = 0;
                init_switch_host(gamepad, address, instance);
            }
            else if (init_state_ == InitState::LED && reply_sub == SwitchPro::CMD::LED)
            {
                init_state_ = InitState::DONE;
                pending_subcmd_ack_ = 0;
                init_step_retries_ = 0;
            }
        }
        else if (len >= 2 &&
                 (report[0] == SwitchPro::REPORT_ID_STANDARD ||
                  report[0] == SwitchPro::REPORT_ID_FULL_ALT))
        {
            /* Decode live input even while final IMU/LED subcommands finish. */
            apply_standard_input(gamepad, report, len);
            init_switch_host(gamepad, address, instance);
            (void)switch_rearm_hid_in(address, instance);
            return;
        }
        else
        {
            init_switch_host(gamepad, address, instance);
        }

        (void)switch_rearm_hid_in(address, instance);
        return;
    }

    /* Self-heal if wireless controller wakes up or re-pairs in simple 0x3F mode after dongle init. */
    if (len >= 1 && report[0] == 0x3F)
    {
        init_state_ = InitState::IMU;
        init_step_retries_ = 0;
        slot.last_cmd_ms = 0;
        init_switch_host(gamepad, address, instance);
        (void)switch_rearm_hid_in(address, instance);
        return;
    }

    apply_standard_input(gamepad, report, len);

    /* Self-heal if wireless controller connected late and is streaming 0x30 with IMU still asleep. */
    if (slot.zero_imu_frames >= kZeroImuWakeThreshold)
    {
        const uint32_t now = board_api::ms_since_boot();
        if ((now - slot.last_cmd_ms) >= kZeroImuWakeIntervalMs && tuh_hid_send_ready(address, instance))
        {
            if (slot.wake_send_mode_next)
            {
                const uint8_t mode = SwitchPro::CMD::FULL_REPORT_MODE;
                (void)send_subcmd_report(address, instance, SwitchPro::SUBCMD_SET_MODE, &mode, 1);
            }
            else
            {
                const uint8_t enable = 1;
                (void)send_subcmd_report(address, instance, SwitchPro::CMD::GYRO, &enable, 1);
            }
            slot.wake_send_mode_next = !slot.wake_send_mode_next;
            slot.last_cmd_ms = now;
        }
    }

    (void)switch_rearm_hid_in(address, instance);
}

bool SwitchProHost::send_feedback(Gamepad& gamepad, uint8_t address, uint8_t instance)
{
    uint16_t vid = 0;
    uint16_t pid = 0;
    tuh_vid_pid_get(address, &vid, &pid);
    const bool switch2_device = is_switch2_usb_family(vid, pid);

    if (switch2_device && switch2_bringup_active_)
    {
        switch2_poll_bringup();
        service_usb_host();
        return false;
    }

    if (switch2_device && switch2_bringup_pending_retry_)
    {
        switch2_bringup_pending_retry_ = false;
        switch2_start_bringup();
        service_usb_host();
        return false;
    }

    if (init_state_ != InitState::DONE)
    {
        init_switch_host(gamepad, address, instance);
        tuh_hid_receive_report(address, instance);
        service_usb_host();
        return false;
    }

    const uint32_t now_ms = board_api::ms_since_boot();

    if (switch2_device)
    {
        if (now_ms - switch2_last_bulk_keepalive_ms_ >= 2000U)
        {
            switch2_last_bulk_keepalive_ms_ = now_ms;
            switch2_send_bulk_keepalive();
        }
    }

    SlotRuntimeState& slot = get_slot_runtime(idx_);

    /* Rate-limit RUMBLE_ONLY packets so PIO-USB and the 2.4GHz RF link are never flooded. */
    Gamepad::PadOut gp_out = gamepad.get_pad_out();
    const bool rumble_changed = (gp_out.rumble_l != slot.last_rumble_l) || (gp_out.rumble_r != slot.last_rumble_r);
    const bool rumble_active = (gp_out.rumble_l > 0) || (gp_out.rumble_r > 0);
    const uint32_t min_interval = rumble_active ? kActiveRumbleIntervalMs : kIdleKeepaliveIntervalMs;

    if (!rumble_changed && (now_ms - slot.last_rumble_ms) < min_interval)
    {
        return false;
    }

    if (!tuh_hid_send_ready(address, instance))
    {
        return false;
    }

    /* Do not step on a recent zero-IMU wake subcommand. */
    if (slot.zero_imu_frames >= kZeroImuWakeThreshold && (now_ms - slot.last_cmd_ms) < 100U)
    {
        return false;
    }

    std::memset(&out_report_, 0, sizeof(out_report_));
    out_report_.command = SwitchPro::CMD::RUMBLE_ONLY;
    out_report_.sequence_counter = get_output_sequence_counter();
    fill_neutral_rumble(out_report_);

    if (gp_out.rumble_l > 0)
    {
        uint8_t amplitude_l = static_cast<uint8_t>(((gp_out.rumble_l / 255.0f) * 0.8f + 0.5f) * (0xC0 - 0x40) + 0x40);
        out_report_.rumble_l[0] = amplitude_l;
        out_report_.rumble_l[1] = 0x88;
        out_report_.rumble_l[2] = amplitude_l / 2;
        out_report_.rumble_l[3] = 0x61;
    }

    if (gp_out.rumble_r > 0)
    {
        uint8_t amplitude_r = static_cast<uint8_t>(((gp_out.rumble_r / 255.0f) * 0.8f + 0.5f) * (0xC0 - 0x40) + 0x40);
        out_report_.rumble_r[0] = amplitude_r;
        out_report_.rumble_r[1] = 0x88;
        out_report_.rumble_r[2] = amplitude_r / 2;
        out_report_.rumble_r[3] = 0x61;
    }

    if (tuh_hid_send_report(address, instance, 0, &out_report_, 10))
    {
        slot.last_rumble_l = gp_out.rumble_l;
        slot.last_rumble_r = gp_out.rumble_r;
        slot.last_rumble_ms = now_ms;
        return true;
    }
    return false;
}
