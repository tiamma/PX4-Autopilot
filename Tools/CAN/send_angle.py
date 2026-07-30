#!/usr/bin/env python3
"""Send a single CAN angle command frame for the can_angle_control driver.

Usage:
    python3 send_angle.py

Modify the variables at the top of the file to change the angle, interface, etc.
"""

import sys

# ---------------------------------------------------------------------------
# User-modifiable variables
# ---------------------------------------------------------------------------
ANGLE_DEG = 10.0          # Angle in degrees (setpoint for 0x200, feedback for 0x201)
ENABLE = False             # Enable flag sent to the actuator (only used for 0x200)
COUNTER = 0               # Frame counter (0..255)
CAN_ID = 0x201            # Feedback frame ID (CA_CTRL_RX_ID) for testing FCU input

INTERFACE = "slcan"       # "slcan" uses a serial USB-CAN adapter, "socketcan" uses a slcan0 interface
PORT = "/dev/ttyACM2"     # Serial port for slcan (only used when INTERFACE == "slcan")
CHANNEL = "slcan0"        # SocketCAN channel (only used when INTERFACE == "socketcan")
BITRATE = 1000000         # CAN bitrate

def checksum(data):
    """Checksum is the low 8 bits of the sum of D0..D5."""
    return sum(data[:6]) & 0xFF


def make_frame(angle_deg, enable=True, counter=0):
    """Build the 8-byte command frame for the can_angle_control protocol."""
    raw = int(round(angle_deg * 100))
    raw &= 0xFFFF  # keep as unsigned 16-bit
    d0 = raw & 0xFF
    d1 = (raw >> 8) & 0xFF
    d2 = 1 if enable else 0
    d3 = counter & 0xFF
    data = [d0, d1, d2, d3, 0, 0, 0, 0]
    data[6] = checksum(data)
    return data


def send_slcan(port, can_id, data):
    """Send a standard CAN frame over an SLCAN serial adapter."""
    import serial
    import time

    ser = serial.Serial(port, 115200, timeout=1)

    # SLCAN initialization: close, set 1 Mbps, open bus
    init_commands = ['C\r', 'S8\r', 'O\r']
    for cmd in init_commands:
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.05)

    hex_data = ''.join(f'{b:02X}' for b in data)
    msg = f't{can_id:03X}{len(data):X}{hex_data}\r'
    ser.write(msg.encode('ascii'))
    ser.flush()
    print(f'Sent via SLCAN ({port}): {msg.strip()}')
    ser.close()

def send_socketcan(channel, can_id, data, bitrate):
    """Send a standard CAN frame over a SocketCAN interface."""
    import can
    bus = can.interface.Bus(channel, bustype='socketcan', bitrate=bitrate)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f'Sent via SocketCAN ({channel}): {msg}')
    bus.shutdown()


def main():
    data = make_frame(ANGLE_DEG, ENABLE, COUNTER)
    print(f'Angle: {ANGLE_DEG:.2f}°, frame bytes: {" ".join(f"{b:02X}" for b in data)}')

    if INTERFACE == "slcan":
        send_slcan(PORT, CAN_ID, data)
    elif INTERFACE == "socketcan":
        send_socketcan(CHANNEL, CAN_ID, data, BITRATE)
    else:
        print(f'Unsupported INTERFACE: {INTERFACE}', file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
