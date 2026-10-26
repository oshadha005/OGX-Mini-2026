#ifndef _MOTION_IMU_H_
#define _MOTION_IMU_H_

#include <cstdint>
#include <cstring>

#include "Gamepad/Gamepad.h"

namespace MotionImu
{

constexpr int32_t kDs4AccRange = 4 * 8192;
constexpr int32_t kDs4GyroRange = 2048 * 1024;
constexpr int32_t kRawSensDenom = 32767;

constexpr int32_t kSwitchDefaultAccelScale = 16384;
constexpr int32_t kSwitchDefaultGyroScale = 13371;
constexpr int32_t kSwitchGyroPrecScale = 1000;
constexpr int32_t kWiiAccelCountsPerG = 0x6C;
constexpr int32_t kDs4UnitsPerG = 8192;

#pragma pack(push, 1)
struct SwitchImuSample
{
    int16_t accel[3];
    int16_t gyro[3];
};
#pragma pack(pop)

static_assert(sizeof(SwitchImuSample) == 12);

inline int16_t clamp_i16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}

inline bool switch_imu_sample_is_zero(const uint8_t* p)
{
    for (size_t i = 0; i < sizeof(SwitchImuSample); ++i) {
        if (p[i] != 0) {
            return false;
        }
    }
    return true;
}

/**
 * Extract 6-axis IMU from Switch 0x30/0x31 payload (payload = report + 2).
 * payload[0..9]  = battery + buttons(3) + sticks(6) (report[2..11])
 * payload[10]    = vibrator input byte              (report[12])
 * payload[11..22]= IMU sample 0                     (report[13..24])
 * payload[23..34]= IMU sample 1                     (report[25..36])
 * payload[35..46]= IMU sample 2                     (report[37..48])
 * If all 3 samples are populated, average them to preserve full sub-frame motion and cut noise.
 */
inline bool switch_usb_extract_sample(SwitchImuSample& out, const uint8_t* payload, uint16_t payload_len)
{
    constexpr uint16_t kInReportSize = 11;
    constexpr uint16_t kImuSampleSize = sizeof(SwitchImuSample);

    if (payload_len < kInReportSize + kImuSampleSize) {
        std::memset(&out, 0, sizeof(out));
        return false;
    }

    SwitchImuSample s0{};
    std::memcpy(&s0, payload + kInReportSize, sizeof(s0));

    if (payload_len >= kInReportSize + (3 * kImuSampleSize)) {
        const uint8_t* p1 = payload + kInReportSize + kImuSampleSize;
        const uint8_t* p2 = payload + kInReportSize + (2 * kImuSampleSize);
        if (!switch_imu_sample_is_zero(p1) && !switch_imu_sample_is_zero(p2)) {
            SwitchImuSample s1{};
            SwitchImuSample s2{};
            std::memcpy(&s1, p1, sizeof(s1));
            std::memcpy(&s2, p2, sizeof(s2));
            for (int i = 0; i < 3; ++i) {
                out.accel[i] = static_cast<int16_t>(
                    (static_cast<int32_t>(s0.accel[i]) + s1.accel[i] + s2.accel[i]) / 3);
                out.gyro[i] = static_cast<int16_t>(
                    (static_cast<int32_t>(s0.gyro[i]) + s1.gyro[i] + s2.gyro[i]) / 3);
            }
            return true;
        }
    }

    out = s0;
    return true;
}

inline int32_t ds4_ds5_usb_raw_to_units(int16_t raw, int32_t range)
{
    return static_cast<int32_t>((static_cast<int64_t>(range) * static_cast<int64_t>(raw)) / kRawSensDenom);
}

inline void fill_from_ds4_usb_report(int32_t accel[3], int32_t gyro[3], const uint8_t* report)
{
    int16_t raw_g[3];
    int16_t raw_a[3];
    std::memcpy(&raw_g[0], &report[13], 2);
    std::memcpy(&raw_g[1], &report[15], 2);
    std::memcpy(&raw_g[2], &report[17], 2);
    std::memcpy(&raw_a[0], &report[19], 2);
    std::memcpy(&raw_a[1], &report[21], 2);
    std::memcpy(&raw_a[2], &report[23], 2);
    for (int i = 0; i < 3; i++) {
        gyro[i] = ds4_ds5_usb_raw_to_units(raw_g[i], kDs4GyroRange);
        accel[i] = ds4_ds5_usb_raw_to_units(raw_a[i], kDs4AccRange);
    }
}

inline void fill_from_ds5_usb_raw(int32_t accel[3], int32_t gyro[3], const uint16_t raw_accel[3], const uint16_t raw_gyro[3])
{
    for (int i = 0; i < 3; i++) {
        gyro[i] = ds4_ds5_usb_raw_to_units(static_cast<int16_t>(raw_gyro[i]), kDs4GyroRange);
        accel[i] = ds4_ds5_usb_raw_to_units(static_cast<int16_t>(raw_accel[i]), kDs4AccRange);
    }
}

inline void fill_from_switch_usb_payload(int32_t accel[3], int32_t gyro[3], const uint8_t* payload, uint16_t payload_len)
{
    SwitchImuSample raw{};
    if (!switch_usb_extract_sample(raw, payload, payload_len)) {
        for (int i = 0; i < 3; i++) {
            accel[i] = 0;
            gyro[i] = 0;
        }
        return;
    }

    for (int i = 0; i < 3; i++) {
        /* Switch accel is +-8G (4096 LSB/G); DS4 expects +-4G (8192 LSB/G). Clamp in int32 first. */
        const int16_t ds4_accel = clamp_i16(static_cast<int32_t>(raw.accel[i]) * 2);
        accel[i] = ds4_ds5_usb_raw_to_units(ds4_accel, kDs4AccRange);
        /* Both Switch Pro and DS4 gyros use +-2000 dps signed 16-bit full scale (~16.38 LSB/dps). */
        gyro[i] = ds4_ds5_usb_raw_to_units(raw.gyro[i], kDs4GyroRange);
    }
}

inline bool switch_usb_imu_unchanged(const uint8_t* prev_imu, const uint8_t* payload, uint16_t payload_len)
{
    SwitchImuSample cur{};
    if (!switch_usb_extract_sample(cur, payload, payload_len)) {
        return true;
    }
    return std::memcmp(prev_imu, &cur, sizeof(cur)) == 0;
}

inline void fill_from_wii_bt(int32_t accel[3], int32_t gyro[3], const int32_t wii_accel[3])
{
    for (int i = 0; i < 3; i++) {
        accel[i] = static_cast<int32_t>((static_cast<int64_t>(wii_accel[i]) * kDs4UnitsPerG) / kWiiAccelCountsPerG);
        gyro[i] = 0;
    }
}

struct WiiPseudoGyroState
{
    int32_t filt_accel[3];
    int32_t prev_filt[3];
    uint32_t prev_ms;
    bool valid;
};

inline void apply_wii_pseudo_gyro(int32_t accel[3], int32_t gyro[3], WiiPseudoGyroState& st, uint32_t now_ms)
{
    gyro[0] = gyro[1] = gyro[2] = 0;

    if (!st.valid) {
        for (int i = 0; i < 3; i++) {
            st.filt_accel[i] = accel[i];
            st.prev_filt[i] = accel[i];
        }
        st.prev_ms = now_ms;
        st.valid = true;
        return;
    }

    for (int i = 0; i < 3; i++) {
        st.filt_accel[i] = (st.filt_accel[i] * 2 + accel[i]) / 3;
        accel[i] = st.filt_accel[i];
    }

    uint32_t dt = now_ms - st.prev_ms;
    if (dt < 1) {
        return;
    }
    if (dt > 40) {
        dt = 40;
    }

    constexpr int32_t kMaxRaw = 1;
    for (int i = 0; i < 3; i++) {
        int32_t d = st.filt_accel[i] - st.prev_filt[i];
        st.prev_filt[i] = st.filt_accel[i];

        if (d > -20 && d < 20) {
            continue;
        }

        int32_t rate = d / (static_cast<int32_t>(dt) * 5);
        if (rate > kMaxRaw) {
            rate = kMaxRaw;
        } else if (rate < -kMaxRaw) {
            rate = -kMaxRaw;
        }
        gyro[i] = rate;
    }
    st.prev_ms = now_ms;
}

inline void remap_to_ds4_playing_frame(uint8_t motion_source, int32_t accel[3], int32_t gyro[3])
{
    const int32_t ix = accel[0];
    const int32_t iy = accel[1];
    const int32_t iz = accel[2];
    const int32_t gx = gyro[0];
    const int32_t gy = gyro[1];
    const int32_t gz = gyro[2];

    switch (motion_source) {
        case Gamepad::PadIn::MOTION_SRC_WII_BT:
            accel[0] = -ix;
            accel[1] = iz;
            accel[2] = -iy;
            gyro[0] = -gx;
            gyro[1] = gz;
            gyro[2] = -gy;
            break;
        case Gamepad::PadIn::MOTION_SRC_SWITCH_PRO:
        case Gamepad::PadIn::MOTION_SRC_SWITCH_USB:
            accel[0] = -iy;
            accel[1] = -ix;
            accel[2] = -iz;
            gyro[0] = -gy;
            gyro[1] = -gx;
            gyro[2] = -gz;
            break;
        default:
            break;
    }
}

inline void encode_switch_imu_sample(SwitchImuSample& out, const Gamepad::PadIn& gp_in)
{
    if (!gp_in.has_motion()) {
        out.accel[0] = 0;
        out.accel[1] = 0;
        out.accel[2] = static_cast<int16_t>(kSwitchDefaultAccelScale);
        out.gyro[0] = out.gyro[1] = out.gyro[2] = 0;
        return;
    }

    int32_t ax = gp_in.accel[0];
    int32_t ay = gp_in.accel[1];
    int32_t az = gp_in.accel[2];
    int32_t gx = gp_in.gyro[0];
    int32_t gy = gp_in.gyro[1];
    int32_t gz = gp_in.gyro[2];

    int32_t sx = 0;
    int32_t sy = 0;
    int32_t sz = 0;
    int32_t sgx = 0;
    int32_t sgy = 0;
    int32_t sgz = 0;

    switch (gp_in.motion_source) {
        case Gamepad::PadIn::MOTION_SRC_SWITCH_PRO:
            out.accel[0] = clamp_i16(ax);
            out.accel[1] = clamp_i16(ay);
            out.accel[2] = clamp_i16(az);
            out.gyro[0] = clamp_i16(gx / kSwitchGyroPrecScale);
            out.gyro[1] = clamp_i16(gy / kSwitchGyroPrecScale);
            out.gyro[2] = clamp_i16(gz / kSwitchGyroPrecScale);
            return;
        case Gamepad::PadIn::MOTION_SRC_SWITCH_USB:
            out.accel[0] = clamp_i16(ax / 2);
            out.accel[1] = clamp_i16(ay / 2);
            out.accel[2] = clamp_i16(az / 2);
            out.gyro[0] = clamp_i16(static_cast<int32_t>((static_cast<int64_t>(gx) * kRawSensDenom) / kDs4GyroRange));
            out.gyro[1] = clamp_i16(static_cast<int32_t>((static_cast<int64_t>(gy) * kRawSensDenom) / kDs4GyroRange));
            out.gyro[2] = clamp_i16(static_cast<int32_t>((static_cast<int64_t>(gz) * kRawSensDenom) / kDs4GyroRange));
            return;
        case Gamepad::PadIn::MOTION_SRC_WII_BT:
            sx = -az;
            sy = ax;
            sz = ay;
            out.accel[0] = clamp_i16((sx * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
            out.accel[1] = clamp_i16((sy * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
            out.accel[2] = clamp_i16((sz * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
            out.gyro[0] = clamp_i16(-gz);
            out.gyro[1] = clamp_i16(gx);
            out.gyro[2] = clamp_i16(gy);
            return;
        default:
            sx = -ay;
            sy = -ax;
            sz = -az;
            sgx = -gy;
            sgy = -gx;
            sgz = -gz;
            break;
    }

    out.accel[0] = clamp_i16((sx * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
    out.accel[1] = clamp_i16((sy * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
    out.accel[2] = clamp_i16((sz * kSwitchDefaultAccelScale) / kDs4UnitsPerG);
    out.gyro[0] = clamp_i16(sgx / kSwitchGyroPrecScale);
    out.gyro[1] = clamp_i16(sgy / kSwitchGyroPrecScale);
    out.gyro[2] = clamp_i16(sgz / kSwitchGyroPrecScale);
}

struct SwitchImuHistory
{
    SwitchImuSample samples[3];
    bool valid;
};

inline void encode_switch_imu_report(uint8_t* dest, const Gamepad::PadIn& gp_in, SwitchImuHistory& hist)
{
    SwitchImuSample sample{};
    encode_switch_imu_sample(sample, gp_in);

    if (gp_in.motion_source == Gamepad::PadIn::MOTION_SRC_WII_BT) {
        SwitchImuSample accel_only = sample;
        accel_only.gyro[0] = accel_only.gyro[1] = accel_only.gyro[2] = 0;
        std::memcpy(dest + 0 * sizeof(sample), &accel_only, sizeof(sample));
        std::memcpy(dest + 1 * sizeof(sample), &accel_only, sizeof(sample));
        std::memcpy(dest + 2 * sizeof(sample), &sample, sizeof(sample));
        hist.valid = false;
        return;
    }

    if (!hist.valid) {
        hist.samples[0] = hist.samples[1] = hist.samples[2] = sample;
        hist.valid = true;
    } else {
        hist.samples[0] = hist.samples[1];
        hist.samples[1] = hist.samples[2];
        hist.samples[2] = sample;
    }
    for (int i = 0; i < 3; i++) {
        std::memcpy(dest + static_cast<size_t>(i) * sizeof(sample), &hist.samples[i], sizeof(sample));
    }
}

} // namespace MotionImu

#endif // _MOTION_IMU_H_
