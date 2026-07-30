#!/usr/bin/env python3
"""Receive and decode CAN angle control frames.

This script mimics the logic in CanAngleControl::receive_feedback() to verify
that feedback frames (0x201) and command frames (0x200) on the bus are valid and
can be parsed.

Usage:
    # First set up a SocketCAN interface, e.g.:
    #   sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyACM2 slcan0
    #   sudo ip link set slcan0 up type can bitrate 1000000
    python3 receive_angle.py
"""

import sys
import can

# ---------------------------------------------------------------------------
# User-modifiable variables
# ---------------------------------------------------------------------------
TX_ID = 0x200             # Command frame ID (CA_CTRL_TX_ID)
RX_ID = 0x201             # Feedback frame ID (CA_CTRL_RX_ID)
CHANNEL = "slcan0"        # SocketCAN interface
BITRATE = 1000000         # CAN bitrate (only used for some interfaces)

DIRECTION = 0             # 0 = normal, 1 = inverted (matches CA_CTRL_DIR)
OFFSET = 0.0              # Zero offset in degrees (matches CA_CTRL_OFFSET)
MIN_ANGLE = -90.0         # Minimum angle in degrees (matches CA_CTRL_MIN)
MAX_ANGLE = 90.0            # Maximum angle in degrees (matches CA_CTRL_MAX)
CHECKSUM_LEN = 6          # Bytes D0..D5 are checksummed


def checksum(data):
    """Compute the same checksum the driver uses: sum of D0..D5, low 8 bits."""
    return sum(data[:CHECKSUM_LEN]) & 0xFF


def decode_angle(data, direction=DIRECTION, offset=OFFSET, min_angle=MIN_ANGLE, max_angle=MAX_ANGLE):
    """Decode D0-D1 as int16 (0.01 deg) and apply direction/offset/clamp."""
    raw = int(data[0] | (data[1] << 8))
    if raw > 32767:
        raw -= 65536

    raw_deg = raw * 0.01
    direction_mult = 1 if direction == 0 else -1
    physical_deg = direction_mult * raw_deg - offset
    physical_deg = max(min(physical_deg, max_angle), min_angle)
    rad = physical_deg * (3.141592653589793 / 180.0)
    return physical_deg, rad


def format_frame(can_id, data):
    """Pretty-print a CAN frame."""
    hex_str = ' '.join(f'{b:02X}' for b in data)
    chksum_ok = checksum(data) == data[6]

    if can_id == RX_ID:
        physical_deg, rad = decode_angle(data)
        error = int(data[4] | (data[5] << 8))
        return (f'Feedback 0x{can_id:03X} [{hex_str}]  '
                f'angle={physical_deg:7.2f}° ({rad:.4f} rad)  '
                f'error=0x{error:04X}  counter={data[3]:02X}  '
                f'checksum={"OK" if chksum_ok else "FAIL"}')

    elif can_id == TX_ID:
        raw = int(data[0] | (data[1] << 8))
        if raw > 32767:
            raw -= 65536
        setpoint_deg = raw * 0.01
        enabled = data[2] != 0
        return (f'Command  0x{can_id:03X} [{hex_str}]  '
                f'setpoint={setpoint_deg:7.2f}°  '
                f'enable={enabled}  counter={data[3]:02X}  '
                f'checksum={"OK" if chksum_ok else "FAIL"}')

    return f'Unknown  0x{can_id:03X} [{hex_str}]'


def main():
    print(f'Listening on {CHANNEL} for IDs 0x{TX_ID:03X} and 0x{RX_ID:03X}...')
    bus = can.interface.Bus(CHANNEL, bustype='socketcan', bitrate=BITRATE)

    try:
        for msg in bus:
            if msg.is_extended_id:
                continue
            if msg.dlc < 8:
                continue

            can_id = msg.arbitration_id & 0x7FF
            if can_id not in (TX_ID, RX_ID):
                continue

            print(format_frame(can_id, msg.data))

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
