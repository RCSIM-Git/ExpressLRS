#include "devImu.h"

#if defined(TARGET_RX)

#include "CRSFRouter.h"
#include <Wire.h>
#include <math.h>
#include "mpu9250.h"
#include "logging.h"

#define IMU_PUBLISH_INTERVAL_MS   10   // 100Hz (10ms)

extern bool i2c_enabled; // set true by rxtx_common.cpp once WebUI I2C pins are configured & Wire.begin() ran
extern int i2c_gpio_sda;
extern int i2c_gpio_scl;

static MPU9250 imu;
static bool imu_initialized = false;
static uint8_t imu_address = 0x68;

static uint8_t diag_err68 = 99;
static uint8_t diag_whoami68 = 0;
static uint8_t diag_err69 = 99;
static uint8_t diag_whoami69 = 0;

static float ahrs_pitch = 0.0f; // radians (-PI to +PI)
static float ahrs_roll  = 0.0f; // radians (-PI to +PI)
static float ahrs_yaw   = 0.0f; // radians (-PI to +PI)
static bool ahrs_initialized = false;
static uint32_t ahrs_last_us = 0;

// Standard CRSF 0x13: timestamp, gyro XYZ, acceleration XYZ, temperature.
typedef struct {
    uint32_t sample_time_us;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t accel_x, accel_y, accel_z;
    int16_t temperature_centi;
} PACKED crsf_sensor_accgyro_local_t;
static_assert(sizeof(crsf_sensor_accgyro_local_t) == 18, "CRSF AccGyro payload must be 18 bytes");

static int16_t Imu_Scale(float value)
{
    return (int16_t)roundf(fmaxf(-32767.0f, fminf(32767.0f, value)));
}

static void Imu_PublishRaw(uint32_t timestamp, int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz, int16_t temperature)
{
    CRSF_MK_FRAME_T(crsf_sensor_accgyro_local_t) frame = {0};
    frame.p.sample_time_us = htobe32(timestamp);
    // MPU configured at +/-250 dps (131 LSB/dps), +/-2g (16384 LSB/g).
    // CRSF positive roll is LEFT; existing AHRS uses positive roll RIGHT.
    constexpr float gyroScale = 32767.0f / (2000.0f * 131.0f);
    constexpr float accelScale = 32767.0f / (16.0f * 16384.0f);
    frame.p.gyro_x = htobe16(Imu_Scale(-gx * gyroScale));
    frame.p.gyro_y = htobe16(Imu_Scale(gy * gyroScale));
    frame.p.gyro_z = htobe16(Imu_Scale(gz * gyroScale));
    frame.p.accel_x = htobe16(Imu_Scale(ax * accelScale));
    frame.p.accel_y = htobe16(Imu_Scale(ay * accelScale));
    frame.p.accel_z = htobe16(Imu_Scale(az * accelScale));
    frame.p.temperature_centi = htobe16(temperature);
    crsfRouter.SetHeaderAndCrc(&frame.h, (crsf_frame_type_e)0x13, CRSF_FRAME_SIZE(sizeof(frame.p)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &frame.h);
}

static void Imu_PublishSample()
{
    if (!imu_initialized)
    {
        if (i2c_enabled)
        {
#if defined(PLATFORM_ESP8266)
            if (i2c_gpio_sda != UNDEF_PIN && i2c_gpio_scl != UNDEF_PIN)
            {
                digitalWrite(i2c_gpio_sda, LOW);
                digitalWrite(i2c_gpio_scl, LOW);
                GPOC = (1 << i2c_gpio_sda) | (1 << i2c_gpio_scl);
            }
#endif
            uint8_t detected_addr = 0;
            if (MPU9250::detect(&detected_addr, &diag_err68, &diag_whoami68, &diag_err69, &diag_whoami69))
            {
                imu_address = detected_addr;
                if (imu.initialize(imu_address))
                {
                    Wire.setClock(400000);
                    imu_initialized = true;
                    ahrs_initialized = false;
                    DBGLN("IMU successfully initialized at addr 0x%02X!", imu_address);
                }
            }
            else if (diag_err68 == 2 && diag_err69 == 2 && i2c_gpio_sda != UNDEF_PIN && i2c_gpio_scl != UNDEF_PIN)
            {
                // Both 0x68 and 0x69 returned NACK.
                // Try swapping SDA and SCL pins in case user swapped CH4 and CH5!
#if defined(PLATFORM_ESP8266)
                digitalWrite(i2c_gpio_sda, LOW);
                digitalWrite(i2c_gpio_scl, LOW);
                GPOC = (1 << i2c_gpio_sda) | (1 << i2c_gpio_scl);
#endif
                Wire.begin(i2c_gpio_scl, i2c_gpio_sda);
                Wire.setClock(400000);
                uint8_t sw_addr = 0, sw_e68 = 99, sw_w68 = 0, sw_e69 = 99, sw_w69 = 0;
                if (MPU9250::detect(&sw_addr, &sw_e68, &sw_w68, &sw_e69, &sw_w69))
                {
                    // Reversed pins succeeded! Commit the swap permanently
                    int tmp = i2c_gpio_sda;
                    i2c_gpio_sda = i2c_gpio_scl;
                    i2c_gpio_scl = tmp;
                    imu_address = sw_addr;
                    diag_err68 = sw_e68; diag_whoami68 = sw_w68;
                    diag_err69 = sw_e69; diag_whoami69 = sw_w69;
                    if (imu.initialize(imu_address))
                    {
                        Wire.setClock(400000);
                        imu_initialized = true;
                        ahrs_initialized = false;
                        DBGLN("IMU auto-pin-swap success! Locked to SDA %d, SCL %d at addr 0x%02X", i2c_gpio_sda, i2c_gpio_scl, imu_address);
                    }
                }
                else
                {
                    // Restore original configuration for next cycle
                    Wire.begin(i2c_gpio_sda, i2c_gpio_scl);
                    Wire.setClock(400000);
                }
            }
        }
        return;
    }

    if (imu_initialized)
    {
        int16_t r_ax = 0, r_ay = 0, r_az = 16384;
        int16_t r_gx = 0, r_gy = 0, r_gz = 0;
        int16_t temperature = 0;
        if (!imu.readAccelGyro(r_ax, r_ay, r_az, r_gx, r_gy, r_gz, &temperature))
        {
            imu_initialized = false; // lost connection, retry next loop
            ahrs_initialized = false;
            return;
        }

        uint32_t now_us = micros();
        Imu_PublishRaw(now_us, r_ax, r_ay, r_az, r_gx, r_gy, r_gz, temperature);
        float dt = 0.01f; // default 10ms (100Hz)
        if (ahrs_last_us != 0)
        {
            uint32_t diff_us = now_us - ahrs_last_us;
            if (diff_us > 1000 && diff_us < 100000)
            {
                dt = (float)diff_us * 1e-6f;
            }
        }
        ahrs_last_us = now_us;

        // Gyro rates to rad/s (+-250 dps full scale -> 131 LSB/dps)
        constexpr float GYRO_TO_RAD = 0.00013323124f; // (1.0f / 131.0f) * (PI / 180.0f)
        float gx_rad = (float)r_gx * GYRO_TO_RAD;
        float gy_rad = (float)r_gy * GYRO_TO_RAD;
        float gz_rad = (float)r_gz * GYRO_TO_RAD;

        // Accel pitch & roll
        float fax = (float)r_ax;
        float fay = (float)r_ay;
        float faz = (float)r_az;

        float pitch_acc = atan2f(-fax, sqrtf(fay * fay + faz * faz));
        float roll_acc  = atan2f(fay, faz);

        if (!ahrs_initialized)
        {
            ahrs_pitch = pitch_acc;
            ahrs_roll  = roll_acc;
            ahrs_yaw   = 0.0f;
            ahrs_initialized = true;
        }
        else
        {
            // Gyro integration
            ahrs_pitch += gy_rad * dt;
            ahrs_roll  += gx_rad * dt;
            ahrs_yaw   += gz_rad * dt;

            // Wrap yaw to [-PI, PI]
            constexpr float PI_F = 3.14159265f;
            if (ahrs_yaw > PI_F) ahrs_yaw -= 2.0f * PI_F;
            else if (ahrs_yaw < -PI_F) ahrs_yaw += 2.0f * PI_F;

            // Accelerometer dynamic acceleration gating:
            // Check if magnitude of acceleration is close to 1.0g (16384 LSB)
            float a_mag_sq = (fax * fax + fay * fay + faz * faz) / (16384.0f * 16384.0f);
            float alpha = 0.98f; // 98% gyro, 2% accel
            if (a_mag_sq < 0.6f || a_mag_sq > 1.4f)
            {
                // Dynamic maneuvers / bumps: trust gyro only
                alpha = 0.999f;
            }

            ahrs_pitch = alpha * ahrs_pitch + (1.0f - alpha) * pitch_acc;
            ahrs_roll  = alpha * ahrs_roll  + (1.0f - alpha) * roll_acc;
        }

        // Send CRSF_FRAMETYPE_ATTITUDE (0x1E) - exactly like Betaflight!
        // Pitch/Roll/Yaw in radians * 10000 (signed 16-bit)
        CRSF_MK_FRAME_T(crsf_sensor_attitude_t) crsfAtt = {0};
        crsfAtt.p.pitch = htobe16((int16_t)roundf(ahrs_pitch * 10000.0f));
        crsfAtt.p.roll  = htobe16((int16_t)roundf(ahrs_roll * 10000.0f));
        crsfAtt.p.yaw   = htobe16((int16_t)roundf(ahrs_yaw * 10000.0f));

        crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfAtt, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)));
        crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfAtt.h);
    }
}

static bool initialize()
{
    return true;
}

static int start()
{
    imu_initialized = false;
    ahrs_initialized = false;
    ahrs_last_us = 0;
    return IMU_PUBLISH_INTERVAL_MS;
}

static int timeout()
{
    Imu_PublishSample();
    return IMU_PUBLISH_INTERVAL_MS;
}

device_t Imu_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
    .subscribe = EVENT_NONE
};

#endif
