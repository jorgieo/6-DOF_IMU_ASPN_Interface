/*
  Modified from: example1-basic

  This example shows the basic settings and functions for retrieving accelerometer
  and gyroscopic data.
  Please refer to the header file for more possible settings, found here:
  ..\SparkFun_6DoF_ISM330DHCX_Arduino_Library\src\sfe_ism330dhcx_defs.h

  Written by Elias Santistevan @ SparkFun Electronics, August 2022

  Product:
    https://www.sparkfun.com/products/19764

  Repository:
    https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT
  License (http://opensource.org/licenses/MIT).

  Modified by: Jorge Otero

  Outputs ASPN 2023 ICD measurement_IMU (SAMPLED mode) binary messages
  over Serial, wrapped with a minimal framing layer:
    [0xAA][0x55][LENGTH:uint16-LE][PAYLOAD:79 bytes][CRC16-CCITT:uint16-LE]
*/

#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include <math.h>
#include <string.h>

// WGS84 Mean Value of Normal Gravity in m/s^2
#define GAMMA_bar 9.7976432223

// Standard Gravity in m/s^2
#define G 9.80665

// ASPN header identifiers
#define ASPN_VENDOR_ID           0x494D5500UL        // "IMU\0"
#define ASPN_DEVICE_ID           0x494D333333304843ULL // "IM330HC"
#define ASPN_CTX_MEASUREMENT     0x00000001UL
#define ASPN_CTX_METADATA        0x00000002UL
#define ASPN_IMU_SAMPLED         1UL

// Framing sync bytes
#define SYNC0  0xAA
#define SYNC1  0x55

// measurement_IMU payload (no integrity entries): 79 bytes
// Framed: 2 sync + 2 length + 79 payload + 2 CRC = 85 bytes
#define ASPN_PAYLOAD_LEN   79
#define PACKET_LEN         85

// metadata_IMU payload layout (bytes):
//   type_metadataheader:
//     header (type_header)           18
//     sensor_description_len uint16   2
//     sensor_description      19 ch  19
//     delta_t_nom float64             8
//     timestamp_clock_id uint8        1
//     digits_of_precision uint8       1
//   time_of_validity int64            8
//   type_mounting:
//     lever_arm float64[3]           24
//     lever_arm_sigma float64[3]     24
//     (orientation fields omitted — mounting unknown)
//   error_model enum 32-byte str     32
//   num_error_model_params uint16     2
//   error_model_params float64[14]  112
//   Total                           251
#define META_SENSOR_DESC    "SparkFun ISM330DHCX"
#define META_SENSOR_DESC_LEN 19U
#define META_PAYLOAD_LEN    251
#define META_PACKET_LEN     257   // 2+2+251+2

// ISM330DHCX BASIC error model (14 params).
// Values from datasheet DS13225. Update with calibrated figures if available.
static const double META_ERROR_PARAMS[14] = {
  0.392,      // [0]  accel_bias_sigma          m/s²      (40 mg × 9.7976)
  0.000,      // [1]  accel_bias_mean            m/s²
  0.000,      // [2]  accel_tc_bias_sigma        m/s²      (not characterised)
  0.000,      // [3]  accel_tc_bias_time_const   s
  0.000,      // [4]  accel_scale_factor_mean    ppm
  5000.0,     // [5]  accel_scale_factor_sigma   ppm       (±0.5 % sensitivity tol.)
  7.84e-4,    // [6]  velocity_random_walk       m/s/√s    (80 µg/√Hz × 9.7976)
  5.82e-3,    // [7]  gyro_bias_sigma            rad/s     (±1 dps ÷ 3 × π/180)
  0.000,      // [8]  gyro_bias_mean             rad/s
  0.000,      // [9]  gyro_tc_bias_sigma         rad/s     (not characterised)
  0.000,      // [10] gyro_tc_bias_time_const    s
  0.000,      // [11] gyro_scale_factor_mean     ppm
  3000.0,     // [12] gyro_scale_factor_sigma    ppm       (±0.3 % sensitivity tol.)
  1.13e-4,    // [13] angular_random_walk        rad/√s    (6.5 mdps/√Hz × π/180)
};

SparkFun_ISM330DHCX myISM;

sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

uint16_t seq_id = 0;

// Framed packet buffer
uint8_t packet[PACKET_LEN];


// ---------------------------------------------------------------------------
// Pack helpers (little-endian)
// ---------------------------------------------------------------------------

static void packU16(uint8_t *buf, uint16_t v) {
  buf[0] = (uint8_t)(v);
  buf[1] = (uint8_t)(v >> 8);
}

static void packU32(uint8_t *buf, uint32_t v) {
  buf[0] = (uint8_t)(v);
  buf[1] = (uint8_t)(v >> 8);
  buf[2] = (uint8_t)(v >> 16);
  buf[3] = (uint8_t)(v >> 24);
}

static void packU64(uint8_t *buf, uint64_t v) {
  for (int i = 0; i < 8; i++) {
    buf[i] = (uint8_t)(v >> (8 * i));
  }
}

static void packI64(uint8_t *buf, int64_t v) {
  packU64(buf, (uint64_t)v);
}

// ESP32 is little-endian and uses IEEE 754 natively — safe to memcpy
static void packF64(uint8_t *buf, double v) {
  memcpy(buf, &v, 8);
}

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect)
static uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}


// ---------------------------------------------------------------------------
// Build and send a framed ASPN measurement_IMU SAMPLED packet
// ---------------------------------------------------------------------------

static void sendAspnImu(double ax, double ay, double az,
                        double gx, double gy, double gz) {
  // --- Build 79-byte ASPN payload into packet[4..82] ---
  uint8_t *p = &packet[4];

  // type_header (18 bytes)
  packU32(p + 0,  ASPN_VENDOR_ID);
  packU64(p + 4,  ASPN_DEVICE_ID);
  packU32(p + 12, ASPN_CTX_MEASUREMENT);
  packU16(p + 16, seq_id);

  // time_of_validity.elapsed_nsec (int64, nanoseconds since boot)
  packI64(p + 18, (int64_t)micros() * 1000LL);

  // IMU_type = SAMPLED (uint32)
  packU32(p + 26, ASPN_IMU_SAMPLED);

  // meas_accel[3] in m/s² (float64 each)
  packF64(p + 30, ax);
  packF64(p + 38, ay);
  packF64(p + 46, az);

  // meas_gyro[3] in rad/s (float64 each)
  packF64(p + 54, gx);
  packF64(p + 62, gy);
  packF64(p + 70, gz);

  // num_integrity = 0
  p[78] = 0;

  // --- Frame wrapper ---
  packet[0] = SYNC0;
  packet[1] = SYNC1;
  packU16(&packet[2], (uint16_t)ASPN_PAYLOAD_LEN);

  uint16_t crc = crc16(&packet[4], ASPN_PAYLOAD_LEN);
  packU16(&packet[4 + ASPN_PAYLOAD_LEN], crc);

  Serial.write(packet, PACKET_LEN);

  seq_id++;
}


// ---------------------------------------------------------------------------
// Send metadata_IMU once at startup
// ---------------------------------------------------------------------------

static void sendAspnMetadata() {
  uint8_t pkt[META_PACKET_LEN];
  uint8_t *p = &pkt[4];
  uint16_t off = 0;

  // --- type_metadataheader ---

  // type_header (18 bytes), context_id = metadata
  packU32(p + off, ASPN_VENDOR_ID);       off += 4;
  packU64(p + off, ASPN_DEVICE_ID);       off += 8;
  packU32(p + off, ASPN_CTX_METADATA);    off += 4;
  packU16(p + off, 0);                    off += 2; // sequence_id=0

  // sensor_description: uint16 length + ASCII bytes (no null terminator)
  packU16(p + off, META_SENSOR_DESC_LEN); off += 2;
  memcpy(p + off, META_SENSOR_DESC, META_SENSOR_DESC_LEN); off += META_SENSOR_DESC_LEN;

  // delta_t_nom: nominal measurement interval in seconds
  packF64(p + off, 1.0 / 104.0);         off += 8;

  // timestamp_clock_id: 0 = ASPN System Time (monotonic since power-on)
  p[off++] = 0;

  // digits_of_precision: 6 — micros() gives µs resolution
  p[off++] = 6;

  // --- time_of_validity ---
  packI64(p + off, (int64_t)micros() * 1000LL); off += 8;

  // --- type_mounting ---
  // lever_arm[3] = {0,0,0} m (sensor at body-frame origin)
  packF64(p + off, 0.0); off += 8;
  packF64(p + off, 0.0); off += 8;
  packF64(p + off, 0.0); off += 8;

  // lever_arm_sigma[3] = {0,0,0} m (position precisely known)
  packF64(p + off, 0.0); off += 8;
  packF64(p + off, 0.0); off += 8;
  packF64(p + off, 0.0); off += 8;

  // orientation_quaternion and tilt_error_covariance omitted (mounting unknown)

  // --- error_model: null-padded ASCII to 32 bytes ---
  memset(p + off, 0, 32);
  memcpy(p + off, "BASIC", 5);            off += 32;

  // --- num_error_model_params ---
  packU16(p + off, 14);                   off += 2;

  // --- error_model_params (14 × float64) ---
  for (uint8_t i = 0; i < 14; i++) {
    packF64(p + off, META_ERROR_PARAMS[i]); off += 8;
  }

  // --- Frame wrapper ---
  pkt[0] = SYNC0;
  pkt[1] = SYNC1;
  packU16(&pkt[2], (uint16_t)META_PAYLOAD_LEN);

  uint16_t crc = crc16(&pkt[4], META_PAYLOAD_LEN);
  packU16(&pkt[4 + META_PAYLOAD_LEN], crc);

  Serial.write(pkt, META_PACKET_LEN);
}


// ---------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------

float milliG_to_meterPerSecondSquared(float milli_g) {
  return (milli_g / 1000.0) * GAMMA_bar;
}

float millidps_to_radPerSecond(float millidps) {
  return (millidps / 1000.0) * PI / 180.0;
}


// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!myISM.begin()) {
    Serial.println("Did not begin.");
    while (1);
  }

  myISM.deviceReset();
  while (!myISM.getDeviceReset()) {
    delay(1);
  }

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();

  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_4g);

  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_500dps);

  myISM.setAccelFilterLP2();
  myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  myISM.setGyroFilterLP1();
  myISM.setGyroLP1Bandwidth(ISM_MEDIUM);

  myISM.enableTimestamp();

  sendAspnMetadata();
}


// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop() {
  if (myISM.checkStatus()) {
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    double ax = milliG_to_meterPerSecondSquared(accelData.xData);
    double ay = -1.0 * milliG_to_meterPerSecondSquared(accelData.yData);
    double az = -1.0 * milliG_to_meterPerSecondSquared(accelData.zData);

    double gx = millidps_to_radPerSecond(gyroData.xData);
    double gy = -1.0 * millidps_to_radPerSecond(gyroData.yData);
    double gz = -1.0 * millidps_to_radPerSecond(gyroData.zData);

    sendAspnImu(ax, ay, az, gx, gy, gz);
  }
}
