"""
ASPN IMU terminal reader.

Reads framed packets from the 6-DOF_IMU Arduino sketch and prints a live display.

Packet format (custom transport framing):
  [0xAA][0x55][LENGTH:uint16-LE][PAYLOAD][CRC16-CCITT:uint16-LE]

Two message types are distinguished by payload length:

  measurement_IMU (79 bytes) — SAMPLED mode, no integrity entries:
    Offset  Field                   Type     Units
     0      vendor_id               uint32
     4      device_id               uint64
    12      context_id              uint32
    16      sequence_id             uint16
    18      time_of_validity_ns     int64    ns
    26      imu_type (1=SAMPLED)    uint32
    30      accel[0] X              float64  m/s²
    38      accel[1] Y              float64  m/s²
    46      accel[2] Z              float64  m/s²
    54      gyro[0]  X              float64  rad/s
    62      gyro[1]  Y              float64  rad/s
    70      gyro[2]  Z              float64  rad/s
    78      num_integrity (0)       uint8

  metadata_IMU (251 bytes):
    Offset  Field                           Type
     0-17   type_header                     (18 bytes)
    18-19   sensor_description_len          uint16
    20-38   sensor_description              19 bytes ASCII
    39-46   delta_t_nom                     float64   s
    47      timestamp_clock_id              uint8
    48      digits_of_precision             uint8
    49-56   time_of_validity_ns             int64     ns
    57-80   lever_arm[3]                    float64[3] m
    81-104  lever_arm_sigma[3]              float64[3] m
   105-136  error_model (null-padded str)   32 bytes
   137-138  num_error_model_params          uint16
   139-250  error_model_params[14]          float64[14]

Usage:
  python imu_reader.py [PORT] [BAUD]

  PORT defaults to COM4, BAUD defaults to 115200.

Dependencies:
  pip install pyserial
"""

import sys
import struct
import os
import math

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Run: pip install pyserial")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SYNC0 = 0xAA
SYNC1 = 0x55

MEAS_PAYLOAD_LEN = 79
META_PAYLOAD_LEN = 251

KNOWN_LENGTHS = {MEAS_PAYLOAD_LEN, META_PAYLOAD_LEN}

# struct format for measurement_IMU payload (all little-endian, 79 bytes total)
# I=uint32 vendor_id, Q=uint64 device_id, I=uint32 context_id, H=uint16 seq,
# q=int64 t_ns, I=uint32 imu_type, 6d=float64×6 accel+gyro, B=uint8 num_integ
MEAS_FMT = "<IQIHqI6dB"
assert struct.calcsize(MEAS_FMT) == MEAS_PAYLOAD_LEN

# BASIC error model parameter labels (index → name, units)
BASIC_PARAM_LABELS = [
    ("accel_bias_sigma",              "m/s²"),
    ("accel_bias_mean",               "m/s²"),
    ("accel_tc_bias_sigma",           "m/s²"),
    ("accel_tc_bias_time_const",      "s"),
    ("accel_scale_factor_mean",       "ppm"),
    ("accel_scale_factor_sigma",      "ppm"),
    ("velocity_random_walk",          "m/s/√s"),
    ("gyro_bias_sigma",               "rad/s"),
    ("gyro_bias_mean",                "rad/s"),
    ("gyro_tc_bias_sigma",            "rad/s"),
    ("gyro_tc_bias_time_const",       "s"),
    ("gyro_scale_factor_mean",        "ppm"),
    ("gyro_scale_factor_sigma",       "ppm"),
    ("angular_random_walk",           "rad/√s"),
]


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect)
# Must match the Arduino implementation.
# ---------------------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Packet reader — searches for sync, reads framed packet of any known length
# ---------------------------------------------------------------------------

def read_packet(port: serial.Serial) -> tuple[bytes, int] | None:
    """
    Returns (payload_bytes, payload_length), or None on CRC mismatch or
    unrecognised length.  Blocks until sync header is found.
    """
    b = port.read(1)
    while True:
        if b and b[0] == SYNC0:
            b2 = port.read(1)
            if b2 and b2[0] == SYNC1:
                break
            b = b2
        else:
            b = port.read(1)

    length_bytes = port.read(2)
    if len(length_bytes) < 2:
        return None
    (length,) = struct.unpack_from("<H", length_bytes)

    if length not in KNOWN_LENGTHS:
        print(f"[resync] unknown length {length}", file=sys.stderr)
        return None

    payload = port.read(length)
    crc_bytes = port.read(2)
    if len(payload) < length or len(crc_bytes) < 2:
        return None

    (received_crc,) = struct.unpack_from("<H", crc_bytes)
    if received_crc != crc16(payload):
        print(f"[CRC mismatch] seq payload len={length}", file=sys.stderr)
        return None

    return payload, length


# ---------------------------------------------------------------------------
# Message parsers
# ---------------------------------------------------------------------------

def parse_measurement(payload: bytes) -> dict:
    fields = struct.unpack_from(MEAS_FMT, payload)
    vendor_id, device_id, context_id, seq, t_ns, imu_type, \
        ax, ay, az, gx, gy, gz, num_integ = fields
    return dict(
        vendor_id=vendor_id, device_id=device_id, context_id=context_id,
        seq=seq, t_ns=t_ns, imu_type=imu_type,
        ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz,
    )


def parse_metadata(payload: bytes) -> dict:
    off = 0

    # type_header
    vendor_id,  = struct.unpack_from("<I", payload, off); off += 4
    device_id,  = struct.unpack_from("<Q", payload, off); off += 8
    context_id, = struct.unpack_from("<I", payload, off); off += 4
    seq,        = struct.unpack_from("<H", payload, off); off += 2

    # sensor_description (length-prefixed)
    desc_len, = struct.unpack_from("<H", payload, off); off += 2
    sensor_desc = payload[off:off + desc_len].decode("ascii", errors="replace"); off += desc_len

    # delta_t_nom, clock_id, digits_of_precision
    delta_t, = struct.unpack_from("<d", payload, off); off += 8
    clock_id  = payload[off]; off += 1
    digits    = payload[off]; off += 1

    # time_of_validity
    t_ns, = struct.unpack_from("<q", payload, off); off += 8

    # type_mounting
    lever_arm   = struct.unpack_from("<3d", payload, off); off += 24
    lever_sigma = struct.unpack_from("<3d", payload, off); off += 24

    # error_model (32-byte null-padded ASCII)
    error_model = payload[off:off + 32].rstrip(b"\x00").decode("ascii", errors="replace"); off += 32

    # num_error_model_params + params
    num_params, = struct.unpack_from("<H", payload, off); off += 2
    params = struct.unpack_from(f"<{num_params}d", payload, off)

    return dict(
        sensor_desc=sensor_desc, delta_t=delta_t, clock_id=clock_id,
        digits=digits, t_ns=t_ns,
        lever_arm=lever_arm, lever_sigma=lever_sigma,
        error_model=error_model, params=params,
    )


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def clear():
    os.system("cls" if os.name == "nt" else "clear")


def fmt(v: float, width: int = 8, decimals: int = 4) -> str:
    return f"{v:+{width}.{decimals}f}"


def display(meas: dict, meta: dict | None) -> None:
    t_sec = meas["t_ns"] * 1e-9
    vendor_str = meas["vendor_id"].to_bytes(4, "big").decode("ascii", errors="replace")
    device_str = meas["device_id"].to_bytes(8, "big").decode("ascii", errors="replace")

    clear()

    if meta:
        odr = 1.0 / meta["delta_t"] if meta["delta_t"] else 0
        clock_names = {0: "ASPN System Time", 1: "TAI", 2: "UTC", 3: "GPS"}
        clock_str = clock_names.get(meta["clock_id"], f"ID={meta['clock_id']}")
        print(f"  Sensor: {meta['sensor_desc']}  |  ODR: {odr:.1f} Hz  |"
              f"  Clock: {clock_str}  |  Precision: 10^-{meta['digits']} s")
        print(f"  Error model: {meta['error_model']}"
              f"  VRW={meta['params'][6]*1e3:.3f} mm/s/√s"
              f"  ARW={meta['params'][13]*1e4:.2f}×10⁻⁴ rad/√s")
        print(f"  {'─'*60}")

    print(f"  ASPN measurement_IMU  |  seq={meas['seq']:6d}  |  t={t_sec:10.4f} s")
    print(f"  {'─'*60}")
    print(f"  Accel  [m/s²]   X: {fmt(meas['ax'])}   Y: {fmt(meas['ay'])}   Z: {fmt(meas['az'])}")
    print(f"  Gyro   [rad/s]  X: {fmt(meas['gx'])}   Y: {fmt(meas['gy'])}   Z: {fmt(meas['gz'])}")
    print(f"  {'─'*60}")
    print(f"  vendor={vendor_str}  device={device_str}  ctx=0x{meas['context_id']:08X}")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    baud_rate  = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port_name} at {baud_rate} baud …")
    try:
        port = serial.Serial(port_name, baud_rate, timeout=2)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open serial port: {exc}")

    print("Waiting for packets (Ctrl+C to quit) …")

    meta: dict | None = None

    try:
        while True:
            result = read_packet(port)
            if result is None:
                continue

            payload, length = result

            if length == META_PAYLOAD_LEN:
                meta = parse_metadata(payload)
                # Print metadata summary once when received
                clear()
                print(f"  [metadata_IMU received]  Sensor: {meta['sensor_desc']}")
                print(f"  Error model: {meta['error_model']}  "
                      f"({len(meta['params'])} params)")
                for i, (label, unit) in enumerate(BASIC_PARAM_LABELS):
                    if i < len(meta["params"]):
                        print(f"    {label:<42s} {meta['params'][i]:+.4e}  {unit}")
                print()
                print("  Waiting for measurement packets …")

            elif length == MEAS_PAYLOAD_LEN:
                meas = parse_measurement(payload)
                display(meas, meta)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        port.close()


if __name__ == "__main__":
    main()
