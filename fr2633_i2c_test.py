#!/usr/bin/env python3
"""Basic I²C communication test for the MSP430FR2633 using a CH341T adapter.

This script requests raw cycle packets from the MSP430FR2633 CapTIvate firmware
and prints the raw values for each cycle to verify that I²C communication is
working.  It uses the TL_CYCLE_PACKET_CMD (0x01) described in the protocol
specification.  No graphical output is performed.
"""

from __future__ import annotations

from typing import List

try:
    from pych341a import Ch341  # type: ignore
except ImportError as exc:  # pragma: no cover - import check
    raise SystemExit("pych341a library is required: pip install pych341a") from exc


I2C_ADDR = 0x0A
CMD_CYCLE_PACKET = 0x01
SENSOR_ID = 0x00


def parse_cycle(buf: bytes) -> List[int]:
    """Extract raw element values from a cycle packet."""

    values: List[int] = []
    if len(buf) == 22:
        for k in range(4):
            base = 6 + 4 * k
            raw = buf[base + 2] | (buf[base + 3] << 8)
            values.append(raw)
    elif len(buf) == 10:
        raw = buf[8] | (buf[9] << 8)
        values.append(raw)
    else:  # unexpected length
        values.append(-1)
    return values


def main() -> None:
    dev = Ch341()
    dev.open()
    dev.i2c_set_speed(400000)

    try:
        for cycle in range(16):
            dev.i2c_write(I2C_ADDR, bytes([CMD_CYCLE_PACKET, SENSOR_ID, cycle]))
            try:
                buf = dev.i2c_read(I2C_ADDR, 22)
            except OSError:
                buf = dev.i2c_read(I2C_ADDR, 10)
            raws = parse_cycle(buf)
            print(f"Cycle {cycle:02d}: {raws}")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
