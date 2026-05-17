# 6-DOF IMU ASPN Interface — Project Documentation

## Overview

This project reads 6-axis inertial data (3-axis accelerometer + 3-axis gyroscope) from a
STMicroelectronics **ISM330DHCX** sensor connected via I²C to a **SparkFun ESP32 IoT RedBoard**,
and streams the data over USB serial as binary messages that conform to the
**ASPN 2023 Interface Control Document (ICD)**.

A companion Python script (`imu_reader.py`) receives the binary stream, validates packet integrity,
decodes both message types, and renders a live terminal display.

---

## Hardware

| Component | Part |
| ----------- | ------ |
| Microcontroller | SparkFun IoT RedBoard (ESP32) |
| IMU | STMicroelectronics ISM330DHCX |
| Interface (board to sensor) | I²C |
| Host connection | USB Serial, 115200 baud |

---

## Repository Files

| File | Description |
| ------ | ------------- |
| `6-DOF_IMU.ino` | Arduino sketch — reads IMU, emits ASPN binary packets |
| `imu_reader.py` | Python terminal reader — decodes and displays ASPN packets |

---

## ASPN ICD

### What ASPN Is

**ASPN** is not an acronym — it originated from a program titled *All Source Position and
Navigation* but has since evolved beyond that name. It is an open Interface Control Document (ICD)
published at [github.com/Open-PNT/ASPN-ICD](https://github.com/Open-PNT/ASPN-ICD) that defines
schemas for exchanging navigation sensor data between components using YAML-described message
structures with explicit field names, data types, and SI units.

The canonical reference implementation is **ASPN-C**, a C-language representation of the standard.
Compliance is verified by confirming that an implementation can be converted to and from ASPN-C.
An implementation that makes different internal decisions — such as using different units or
coordinate frames — can still be ASPN-compliant as long as it can be converted to a compliant
representation.

ASPN defines *what* data means and *how* it is structured in memory. It does **not** specify a
transport layer — there are no sync bytes, checksums, or baud rates mandated by the standard.

The latter are added by this project as a custom framing layer described below.

### Why ASPN

- Provides unambiguous units (all SI: m/s², rad/s, nanoseconds)
- Separates measurement data (`measurement_IMU`) from sensor characteristics (`metadata_IMU`)
- Defined coordinate conventions and measurement modes (SAMPLED vs. INTEGRATED)
- Enables interoperability with other ASPN-compliant navigation software
- Open standard with an active governance body

### Message Types Used

| ASPN Message | Direction | When Sent |
| --- | --- | --- |
| `metadata_IMU` | Arduino → Host | Once at startup (in `setup()`) |
| `measurement_IMU` | Arduino → Host | Continuously at ~104 Hz |

---

## Serial Transport Framing

ASPN does not mandate a wire format. This project wraps each ASPN payload in a minimal frame to
provide **synchronisation** and **error detection**:

```cpp
[0xAA][0x55][LENGTH : uint16-LE][PAYLOAD : N bytes][CRC16 : uint16-LE]
```

| Field | Size | Description |
| --- | --- | --- |
| Sync bytes | 2 | Fixed `0xAA 0x55` — used to re-align after data loss |
| Length | 2 | Payload byte count, little-endian uint16 |
| Payload | N | ASPN message bytes (79 for measurement, 251 for metadata) |
| CRC16 | 2 | CRC-16/CCITT-FALSE over the payload only |

The **CRC-16/CCITT-FALSE** variant uses polynomial `0x1021`, initial value `0xFFFF`, no bit
reflection, no final XOR. The same algorithm runs on both the Arduino (in C) and the Python reader
to allow end-to-end integrity checking.

---

## ASPN Message Layouts

### `measurement_IMU` — 79-byte payload

Sent at every ISM330DHCX data-ready interrupt (~104 Hz). Encodes one SAMPLED-mode IMU observation.

| Offset | Field | Type | Units | Value |
| --- | --- | --- | --- | --- |
| 0 | `vendor_id` | uint32 | — | `0x494D5500` ("IMU\0") |
| 4 | `device_id` | uint64 | — | `0x494D333333304843` ("IM330HC") |
| 12 | `context_id` | uint32 | — | `0x00000001` (measurement stream) |
| 16 | `sequence_id` | uint16 | — | Increments from 0, wraps at 65535 |
| 18 | `time_of_validity.elapsed_nsec` | int64 | ns | `micros() × 1000` — ns since boot |
| 26 | `IMU_type` | uint32 | — | `1` = SAMPLED |
| 30 | `meas_accel[0]` X | float64 | m/s² | Specific force, X axis |
| 38 | `meas_accel[1]` Y | float64 | m/s² | Specific force, Y axis |
| 46 | `meas_accel[2]` Z | float64 | m/s² | Specific force, Z axis |
| 54 | `meas_gyro[0]` X | float64 | rad/s | Angular rate, X axis |
| 62 | `meas_gyro[1]` Y | float64 | rad/s | Angular rate, Y axis |
| 70 | `meas_gyro[2]` Z | float64 | rad/s | Angular rate, Z axis |
| 78 | `num_integrity` | uint8 | — | `0` — no integrity data |

Full framed packet: **85 bytes**. At 104 Hz: ~8,840 bytes/s (~70,720 bits/s), within 115200 baud
with ~38% headroom.

### `metadata_IMU` — 251-byte payload

Sent once from `setup()` before measurement streaming begins. Describes the sensor, timing
reference, physical mounting, and noise error model.

| Offset | Field | Type | Value |
| -------- | ------- | ------ | ------- |
| 0–17 | `type_header` | 18 bytes | vendor/device/context IDs, seq=0 |
| 18–19 | `sensor_description` length | uint16 | 19 |
| 20–38 | `sensor_description` | ASCII | `"SparkFun ISM330DHCX"` |
| 39–46 | `delta_t_nom` | float64 s | `1/104 ≈ 0.009615` |
| 47 | `timestamp_clock_id` | uint8 | `0` = ASPN System Time (since boot) |
| 48 | `digits_of_precision` | uint8 | `6` (microsecond resolution) |
| 49–56 | `time_of_validity.elapsed_nsec` | int64 ns | Time metadata was generated |
| 57–80 | `lever_arm[3]` | float64[3] m | `{0, 0, 0}` — sensor at body origin |
| 81–104 | `lever_arm_sigma[3]` | float64[3] m | `{0, 0, 0}` — position precisely known |
| 105–136 | `error_model` | 32-byte str | `"BASIC"` null-padded |
| 137–138 | `num_error_model_params` | uint16 | `14` |
| 139–250 | `error_model_params[14]` | float64[14] | ISM330DHCX noise parameters |

The orientation quaternion and tilt error covariance fields defined by `type_mounting` are omitted
because the physical mounting orientation of this sensor is not known at compile time.

Full framed packet: **257 bytes**.

#### BASIC Error Model Parameters

| Index | Name | Units | Value | Source |
| ----- | ---- | ----- | ----- | ------ |
| 0 | `accel_bias_sigma` | m/s² | 0.392 | 40 mg zero-level offset × 9.7976 |
| 1 | `accel_bias_mean` | m/s² | 0.0 | Assumed zero |
| 2 | `accel_tc_bias_sigma` | m/s² | 0.0 | Not characterised |
| 3 | `accel_tc_bias_time_const` | s | 0.0 | Not characterised |
| 4 | `accel_scale_factor_mean` | ppm | 0.0 | Assumed zero |
| 5 | `accel_scale_factor_sigma` | ppm | 5000 | ±0.5% sensitivity tolerance |
| 6 | `velocity_random_walk` | m/s/√s | 7.84×10⁻⁴ | 80 µg/√Hz noise density × 9.7976 |
| 7 | `gyro_bias_sigma` | rad/s | 5.82×10⁻³ | ±1 dps zero-rate level ÷ 3 × π/180 |
| 8 | `gyro_bias_mean` | rad/s | 0.0 | Assumed zero |
| 9 | `gyro_tc_bias_sigma` | rad/s | 0.0 | Not characterised |
| 10 | `gyro_tc_bias_time_const` | s | 0.0 | Not characterised |
| 11 | `gyro_scale_factor_mean` | ppm | 0.0 | Assumed zero |
| 12 | `gyro_scale_factor_sigma` | ppm | 3000 | ±0.3% sensitivity tolerance |
| 13 | `angular_random_walk` | rad/√s | 1.13×10⁻⁴ | 6.5 mdps/√Hz noise density × π/180 |

> These values are derived from the ISM330DHCX datasheet (DS13225). Replace with results from an
> Allan deviation analysis if sensor-specific calibration data is available.

---

## Arduino Sketch — `6-DOF_IMU.ino`

### Preprocessor Constants

| Constant | Value | Description |
| ---------- | ------- | ------------- |
| `GAMMA_bar` | 9.7976432223 | WGS84 mean normal gravity in m/s². Used to convert milliG readings to SI specific force. More geodetically correct than standard gravity for navigation applications. |
| `G` | 9.80665 | Standard gravity in m/s². Defined but not used in the current conversion path; retained for reference. |
| `ASPN_VENDOR_ID` | `0x494D5500` | ASPN `vendor_id` field. ASCII encoding of "IMU\0". Not in the ASPN reserved range (0x23000000–0x23FFFFFF). |
| `ASPN_DEVICE_ID` | `0x494D333333304843` | ASPN `device_id` field. ASCII encoding of "IM330HC", uniquely identifying this device model under this vendor. |
| `ASPN_CTX_MEASUREMENT` | `0x00000001` | ASPN `context_id` for measurement_IMU packets. Allows a receiver to distinguish message streams. |
| `ASPN_CTX_METADATA` | `0x00000002` | ASPN `context_id` for metadata_IMU packets. Separate from the measurement context. |
| `ASPN_IMU_SAMPLED` | `1` | Numeric encoding of the ASPN `IMU_type` SAMPLED enum value (INTEGRATED=0, SAMPLED=1 by declaration order). |
| `SYNC0` / `SYNC1` | `0xAA` / `0x55` | Two-byte sync word prepended to every framed packet. The Python reader scans for this pair to re-align after serial data loss. |
| `ASPN_PAYLOAD_LEN` | `79` | Byte length of the measurement_IMU ASPN payload. |
| `PACKET_LEN` | `85` | Total framed measurement packet size: 2 sync + 2 length + 79 payload + 2 CRC. |
| `META_SENSOR_DESC` | `"SparkFun ISM330DHCX"` | ASCII string written into the `sensor_description` field of `metadata_IMU`. |
| `META_SENSOR_DESC_LEN` | `19` | Byte length of `META_SENSOR_DESC` (no null terminator counted). |
| `META_PAYLOAD_LEN` | `251` | Byte length of the metadata_IMU ASPN payload. |
| `META_PACKET_LEN` | `257` | Total framed metadata packet size: 2 + 2 + 251 + 2. |

### Global Variables

| Variable | Type | Description |
| ---------- | ------ | ------------- |
| `myISM` | `SparkFun_ISM330DHCX` | Driver object for the ISM330DHCX sensor. Provides methods for configuration and data retrieval over I²C. |
| `accelData` | `sfe_ism_data_t` | Struct populated by `myISM.getAccel()`. Fields `xData`, `yData`, `zData` hold raw accelerometer readings in milliG (int16_t scaled to float by the library). |
| `gyroData` | `sfe_ism_data_t` | Struct populated by `myISM.getGyro()`. Fields `xData`, `yData`, `zData` hold raw gyroscope readings in milli-degrees-per-second. |
| `seq_id` | `uint16_t` | Rolling sequence counter included in every measurement_IMU header. Increments by one per packet; wraps from 65535 to 0. Allows a receiver to detect dropped packets. |
| `packet` | `uint8_t[85]` | Global reusable buffer for outgoing measurement packets. Sized to `PACKET_LEN`. Not used for metadata (which uses a local stack buffer). |
| `META_ERROR_PARAMS` | `const double[14]` | Read-only array of ISM330DHCX noise and bias parameters for the ASPN BASIC error model. Indexed 0–13 as defined by the `metadata_IMU` schema. Sourced from datasheet DS13225. |

### Typedef — `sfe_ism_data_t`

Defined in the SparkFun ISM330DHCX library. Represents a single three-axis sample from either
the accelerometer or the gyroscope.

```cpp
struct sfe_ism_data_t {
    float xData;   // X-axis reading (milliG for accel, millidps for gyro)
    float yData;   // Y-axis reading
    float zData;   // Z-axis reading
};
```

### Functions

#### `packU16(uint8_t *buf, uint16_t v)`

Writes a 16-bit unsigned integer into two bytes at `buf` in little-endian order (LSB first).
Used to encode `sequence_id`, payload lengths, CRC values, and the sensor_description length
prefix in metadata.

#### `packU32(uint8_t *buf, uint32_t v)`

Writes a 32-bit unsigned integer into four bytes at `buf` in little-endian order.
Used for ASPN `vendor_id`, `context_id`, and the `IMU_type` enum field.

#### `packU64(uint8_t *buf, uint64_t v)`

Writes a 64-bit unsigned integer into eight bytes at `buf` in little-endian order via a byte loop.
Used for ASPN `device_id`.

#### `packI64(uint8_t *buf, int64_t v)`

Writes a signed 64-bit integer by reinterpreting it as `uint64_t` and delegating to `packU64`.
Used for ASPN `time_of_validity.elapsed_nsec`.

#### `packF64(uint8_t *buf, double v)`

Copies the 8 raw bytes of an IEEE 754 double into `buf` using `memcpy`. This is safe on the ESP32
because it is a little-endian processor using native IEEE 754 representation — the byte layout in
memory is identical to what a little-endian receiver expects.

#### `crc16(const uint8_t *data, uint16_t len) → uint16_t`

Computes a CRC-16/CCITT-FALSE checksum over `len` bytes starting at `data`.

- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- No input/output bit reflection
- No final XOR

Applied to the ASPN payload only (not the sync bytes or length field). The resulting 16-bit value
is appended to the framed packet. The Python reader runs the same algorithm and rejects packets
where the recomputed CRC does not match the received value.

#### `sendAspnImu(double ax, double ay, double az, double gx, double gy, double gz)`

Builds and transmits one framed `measurement_IMU` packet. Steps:

1. Writes the 79-byte ASPN payload into `packet[4..82]` using the pack helpers, populating every
   field in schema order: header identifiers → timestamp → IMU_type → accel array → gyro array →
   num_integrity.
2. Prepends the two sync bytes and the little-endian 16-bit length field into `packet[0..3]`.
3. Computes CRC16 over the payload and appends it to `packet[83..84]`.
4. Calls `Serial.write(packet, 85)` to transmit the complete 85-byte packet atomically.
5. Increments `seq_id`.

The timestamp is captured via `(int64_t)micros() * 1000LL` at the moment of packet construction,
giving nanosecond-resolution elapsed time since boot (with microsecond effective precision).

#### `sendAspnMetadata()`

Builds and transmits one framed `metadata_IMU` packet from a 257-byte stack-allocated buffer.
Called once from `setup()` after sensor initialisation is complete. Steps:

1. Encodes the `type_metadataheader`: standard ASPN header (with `ASPN_CTX_METADATA` context and
   `sequence_id=0`), the length-prefixed sensor description string, nominal measurement interval
   `1/104` s, clock ID `0` (ASPN System Time), and precision `6` (microseconds).
2. Encodes `time_of_validity` using `micros()` at the moment of the call.
3. Encodes `type_mounting` with zero lever arm and zero sigma (sensor at body-frame origin).
   Orientation quaternion and tilt covariance are omitted because the physical mounting orientation
   is not known.
4. Encodes the `error_model` enum as the null-padded ASCII string `"BASIC"` in a 32-byte field.
5. Writes `num_error_model_params = 14` and the 14 double-precision values from
   `META_ERROR_PARAMS`.
6. Applies the same sync + length + CRC framing used for measurement packets, then transmits.

#### `milliG_to_meterPerSecondSquared(float milli_g) → float`

Converts a milliG value from the ISM330DHCX accelerometer into SI specific force (m/s²) using the
WGS84 mean normal gravity constant `GAMMA_bar = 9.7976432223 m/s²`. The factor `GAMMA_bar` is
used in preference to the standard gravity constant `G = 9.80665` because it is the geodetically
correct mean surface gravity value, making it more accurate for navigation applications.

```cpp
result = (milli_g / 1000.0) × GAMMA_bar
```

The Y and Z axes receive a sign inversion in `loop()` to align the sensor frame with the expected
right-hand coordinate convention.

#### `millidps_to_radPerSecond(float millidps) → float`

Converts a milli-degrees-per-second value from the ISM330DHCX gyroscope into SI angular rate
(rad/s).

```cpp
result = (millidps / 1000.0) × π / 180.0
```

The Y and Z axes receive a sign inversion in `loop()` matching the same frame correction applied
to the accelerometer.

#### `setup()`

Runs once on power-on or reset. Performs the following in order:

1. Initialises I²C (`Wire.begin()`) and Serial at 115200 baud.
2. Calls `myISM.begin()` — verifies the sensor is present on the I²C bus; halts with an error
   message if it is not.
3. Issues `myISM.deviceReset()` and waits for the sensor to complete its reset cycle, ensuring a
   clean known state regardless of prior configuration.
4. Configures both sensor axes:
   - Output data rate (ODR): **104 Hz** for both accelerometer and gyroscope.
   - Full-scale range: **±4 g** (accelerometer), **±500 dps** (gyroscope).
5. Enables low-pass filters: LP2 + slope filter at ODR/100 for the accelerometer; LP1 at medium
   bandwidth for the gyroscope. These reduce quantisation noise at the cost of ~2 samples of
   group delay.
6. Enables the ISM330DHCX internal hardware timestamp counter (used by the library; the sketch
   derives its own timestamp from `micros()`).
7. Calls `sendAspnMetadata()` to transmit the one-time sensor descriptor before measurement
   streaming begins.

#### `loop()`

Executes continuously. On each iteration:

1. Calls `myISM.checkStatus()` — polls the sensor's status register. Returns true only when both
   accelerometer and gyroscope data registers have been refreshed since the last read (block data
   update mode ensures the X/Y/Z values are always from the same sample).
2. Reads raw data into `accelData` and `gyroData`.
3. Converts units: milliG → m/s² (accel), millidps → rad/s (gyro), with sign inversion on Y and Z
   to correct the sensor coordinate frame.
4. Calls `sendAspnImu()` with the six converted values.

---

## Python Script — `imu_reader.py`

### Usage

```bash
python imu_reader.py [PORT] [BAUD]
```

- `PORT`: Serial port name. Default `COM4`. Example: `COM3`, `/dev/ttyUSB0`.
- `BAUD`: Baud rate. Default `115200`.

**Dependency:** `pip install pyserial`

### Module-Level Constants

| Constant | Value | Description |
| --- | --- | --- |
| `SYNC0` | `0xAA` | First sync byte. Matches Arduino `SYNC0`. |
| `SYNC1` | `0x55` | Second sync byte. Matches Arduino `SYNC1`. |
| `MEAS_PAYLOAD_LEN` | `79` | Expected payload size of a `measurement_IMU` packet. |
| `META_PAYLOAD_LEN` | `251` | Expected payload size of a `metadata_IMU` packet. |
| `KNOWN_LENGTHS` | `{79, 251}` | Set of valid payload lengths. `read_packet()` rejects any length not in this set. |
| `MEAS_FMT` | `"<IQIHqI6dB"` | `struct` format string for a complete 79-byte measurement payload. The leading `<` selects little-endian decoding. Field order matches the ASPN schema byte-for-byte. Verified at import time via `assert struct.calcsize(MEAS_FMT) == 79`. |
| `BASIC_PARAM_LABELS` | list of 14 tuples | Human-readable `(name, units)` labels for the 14 ASPN BASIC error model parameters. Used to format the metadata parameter table at startup. Index order matches `META_ERROR_PARAMS` in the Arduino sketch. |

### Python Functions

#### `crc16(data: bytes) → int`

Computes CRC-16/CCITT-FALSE over `data`. Identical algorithm to the Arduino `crc16()` function —
polynomial `0x1021`, init `0xFFFF`, no reflection. Called by `read_packet()` to verify every
received payload before decoding.

#### `read_packet(port: serial.Serial) → tuple[bytes, int] | None`

The core packet synchronisation and framing layer. Returns `(payload_bytes, payload_length)` on
success, or `None` if the packet should be discarded.

**Sync scanning:** reads one byte at a time, looking for `SYNC0`. When found, immediately reads
the next byte. If it is `SYNC1`, a potential frame start has been found; otherwise the second byte
becomes the new candidate for `SYNC0`. This sliding-window approach guarantees re-synchronisation
within at most two bytes of a valid frame start, regardless of how many bytes were lost before.

**Length validation:** reads the 2-byte little-endian length field. If the decoded value is not in
`KNOWN_LENGTHS`, the packet is discarded and a resync message is printed to stderr.

**Payload and CRC read:** reads exactly `length` bytes of payload, then 2 bytes of CRC. If fewer
bytes arrive before the serial timeout, `None` is returned.

**CRC verification:** recomputes CRC16 over the payload and compares it to the received value. A
mismatch prints a warning to stderr and returns `None`, preventing corrupt data from reaching the
decoders. All error messages go to stderr so they do not contaminate the terminal display.

#### `parse_measurement(payload: bytes) → dict`

Decodes a 79-byte measurement payload into a Python dictionary using a single
`struct.unpack_from(MEAS_FMT, payload)` call. Returns the following keys:

| Key | Type | Description |
| ----- | ------ | ------------- |
| `vendor_id` | int | ASPN vendor identifier |
| `device_id` | int | ASPN device identifier |
| `context_id` | int | ASPN context identifier |
| `seq` | int | Packet sequence number |
| `t_ns` | int | Timestamp in nanoseconds since boot |
| `imu_type` | int | IMU mode enum (1 = SAMPLED) |
| `ax`, `ay`, `az` | float | Accelerometer X/Y/Z in m/s² |
| `gx`, `gy`, `gz` | float | Gyroscope X/Y/Z in rad/s |

#### `parse_metadata(payload: bytes) → dict`

Decodes the 251-byte metadata payload by walking a byte offset manually through the payload.
Manual offset walking is necessary because the `sensor_description` is a variable-length
length-prefixed string that prevents a single fixed-format `struct.unpack` from covering the whole
message.

Decodes fields in schema order:

1. **`type_header`** (18 bytes): four `struct.unpack_from` calls for vendor_id (I), device_id (Q),
   context_id (I), and sequence_id (H).
2. **`sensor_description`** (variable): reads the 2-byte uint16 length, then slices that many
   bytes from the payload and decodes as ASCII.
3. **`delta_t_nom`** (float64): nominal measurement interval in seconds.
4. **`timestamp_clock_id`** (uint8): integer clock source ID; `0` = ASPN System Time.
5. **`digits_of_precision`** (uint8): timestamp decimal precision exponent.
6. **`time_of_validity`** (int64): nanoseconds since boot when metadata was generated.
7. **`lever_arm`** and **`lever_arm_sigma`** (float64[3] each): sensor position and uncertainty in
   the platform body frame.
8. **`error_model`** (32 bytes): null-padded ASCII string stripped and decoded.
9. **`num_error_model_params`** (uint16) and **`error_model_params`** (float64 × N): noise and
   bias model parameters.

Returns a dictionary with keys `sensor_desc`, `delta_t`, `clock_id`, `digits`, `t_ns`,
`lever_arm`, `lever_sigma`, `error_model`, and `params`.

#### `clear()`

Clears the terminal using the platform-appropriate command (`cls` on Windows, `clear` on
POSIX). Called at the start of every `display()` call to produce an in-place updating display
rather than a scrolling log.

#### `fmt(v: float, width: int = 8, decimals: int = 4) → str`

Formats a floating-point number as a fixed-width signed string with a leading `+` or `-` sign.
The default format produces strings like `+9.7912` or `-0.0032`. Consistent width keeps columns
aligned in the terminal display regardless of sign or magnitude within the expected sensor range.

#### `display(meas: dict, meta: dict | None) → None`

Renders the live sensor display to the terminal. Clears the screen, then prints:

1. **Metadata banner** (only if `meta` is not `None`): one line with sensor name, computed ODR
   (`1/delta_t`), human-readable clock source name, timestamp precision, velocity random walk, and
   angular random walk. This banner persists at the top of every measurement update.
2. **Measurement header**: sequence number and elapsed time in seconds.
3. **Sensor values**: accelerometer (m/s²) and gyroscope (rad/s), X/Y/Z formatted by `fmt()`.
4. **Identity footer**: vendor and device IDs decoded as ASCII strings, context ID in hex.

#### `main()`

Entry point. Parses optional command-line arguments for port name and baud rate, then opens the
serial port. Maintains a single `meta` variable (initially `None`) that is populated the first
time a metadata packet is received and reused for every subsequent display update.

The main loop calls `read_packet()` on every iteration. Depending on the returned payload length:

- **251 bytes (metadata):** calls `parse_metadata()`, stores the result in `meta`, and prints a
  one-time formatted table of all 14 error model parameters with their labels and units to the
  terminal. This gives immediate visibility into the sensor's noise characteristics at startup.
- **79 bytes (measurement):** calls `parse_measurement()` and then `display()` with the current
  `meta` state. The display updates at the sensor's full 104 Hz rate.

Handles `KeyboardInterrupt` cleanly and always closes the serial port in a `finally` block.

---

## Coordinate Frame and Sign Conventions

The ISM330DHCX physical axes are right-hand oriented but may not align with the application's
body frame depending on how the board is mounted. The sketch applies sign inversions to Y and Z
axes of both accelerometer and gyroscope to correct for the specific mounting orientation of this
build:

```cpp
ax =  milliG_to_meterPerSecondSquared(accelData.xData)
ay = -milliG_to_meterPerSecondSquared(accelData.yData)
az = -milliG_to_meterPerSecondSquared(accelData.zData)

gx =  millidps_to_radPerSecond(gyroData.xData)
gy = -millidps_to_radPerSecond(gyroData.yData)
gz = -millidps_to_radPerSecond(gyroData.zData)
```

When the sensor is at rest on a flat surface, the expected ASPN output is approximately:

| Axis | Accel (m/s²) | Gyro (rad/s) |
| --- | --- | --- |
| X | ≈ 0 | ≈ 0 |
| Y | ≈ 0 | ≈ 0 |
| Z | ≈ −9.79 | ≈ 0 |

The negative Z acceleration represents the 1g reaction force (specific force convention: the
sensor measures the non-gravitational force, which on a stationary surface is directed upward,
i.e., away from the Earth's surface, and in ASPN's body-frame Z-down convention this is negative).

---

## ISM330DHCX Sensor Configuration

| Parameter | Setting | Constant |
| ----------- | --------- | ---------- |
| Accelerometer ODR | 104 Hz | `ISM_XL_ODR_104Hz` |
| Accelerometer range | ±4 g | `ISM_4g` |
| Accel low-pass filter | LP2 enabled | `setAccelFilterLP2()` |
| Accel slope filter | ODR/100 | `ISM_LP_ODR_DIV_100` |
| Gyroscope ODR | 104 Hz | `ISM_GY_ODR_104Hz` |
| Gyroscope range | ±500 dps | `ISM_500dps` |
| Gyro low-pass filter | LP1, medium bandwidth | `ISM_MEDIUM` |
| Block data update | Enabled | `setBlockDataUpdate()` |
| Hardware timestamp | Enabled | `enableTimestamp()` |

Block data update (BDU) mode ensures that the upper and lower bytes of each axis register are only
refreshed together when both have been read, preventing torn reads where X/Y/Z values come from
different sample instants.

---

## ASPN Compliance Notes

| Requirement | Status | Notes |
| ----------- | ------ | ----- |
| `measurement_IMU` field order and types | Compliant | All fields match schema; float64, uint32, int64 as specified |
| SAMPLED mode units (m/s², rad/s) | Compliant | Conversions verified against ASPN ICD |
| `vendor_id` not in reserved range | Compliant | `0x494D5500` outside `0x23000000–0x23FFFFFF` |
| `sequence_id` monotonically incrementing | Compliant | Increments by 1, wraps 65535→0 |
| `metadata_IMU` transmitted at startup | Compliant | Sent once before measurement stream |
| Timestamp clock reference declared | Compliant | `timestamp_clock_id = 0` in metadata |
| Orientation quaternion | Omitted | Mounting orientation unknown; field is schema-optional |
| `IMU_type` enum numeric value | Convention | ASPN YAML does not assign numeric values; SAMPLED=1 follows declaration order |
| Endianness | Little-endian | Not mandated by ASPN; matches ESP32 native byte order |
| Transport framing | Custom extension | ASPN does not define a wire format; sync+length+CRC16 added by this project |
