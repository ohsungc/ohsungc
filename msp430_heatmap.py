"""MSP430FR2633 heatmap viewer using a CH341T I²C adapter.

This script polls the MSP430FR2633 CapTIvate MCU for raw cycle data
via I²C and displays the resulting 8×8 matrix as a heatmap using
``pyqtgraph``.

The protocol follows the description:
    * Write 3 bytes ``[0x01, 0x00, cycle]`` to request a cycle packet.
    * Attempt to read 22 bytes (four elements) and fall back to 10 bytes
      (single element) if the read fails.
    * Raw values are little‑endian at offsets ``base+2`` and ``base+3``.

The element index for a cycle ``c`` and element ``k`` (0‑3) is::

    E = 8 * (c >> 1) + 2 * k + (c & 1)

Requirements
------------
``pyqtgraph`` and a Python binding for the CH341 (e.g. ``pych341a``)
must be installed.  A connected MSP430FR2633 running CapTIvate firmware
is expected at I²C address ``0x0A``.
"""

from __future__ import annotations

import sys
from typing import List

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

try:
    from pych341a import Ch341  # type: ignore
except ImportError as exc:  # pragma: no cover - import check
    raise SystemExit(
        "pych341a library is required for CH341 access: pip install pych341a"
    ) from exc


# I²C protocol constants
I2C_ADDR = 0x0A
CMD_CYCLE_PACKET = 0x01
SENSOR_ID = 0x00


def open_device() -> Ch341:
    """Open CH341 device at 400 kHz I²C speed."""

    dev = Ch341()
    dev.open()
    dev.i2c_set_speed(400000)
    return dev


def read_cycle(dev: Ch341, cycle: int) -> bytes:
    """Request and read a cycle packet for ``cycle``."""

    dev.i2c_write(I2C_ADDR, bytes([CMD_CYCLE_PACKET, SENSOR_ID, cycle]))
    try:
        return dev.i2c_read(I2C_ADDR, 22)
    except OSError:
        # Some firmware responds with 10‑byte packets (single element)
        return dev.i2c_read(I2C_ADDR, 10)


def parse_cycle(buf: bytes) -> List[int]:
    """Extract raw element values from a cycle packet."""

    elems: List[int] = []
    if len(buf) == 22:
        for k in range(4):
            base = 6 + 4 * k
            raw = buf[base + 2] | (buf[base + 3] << 8)
            elems.append(raw)
    elif len(buf) == 10:
        raw = buf[8] | (buf[9] << 8)
        elems.append(raw)
    return elems


def read_scan(dev: Ch341) -> List[int]:
    """Read all 16 cycles and return a list of 64 raw values."""

    readings = [0] * 64
    for cycle in range(16):
        buf = read_cycle(dev, cycle)
        elems = parse_cycle(buf)
        for k, raw in enumerate(elems):
            e = 8 * (cycle >> 1) + 2 * k + (cycle & 1)
            readings[e] = raw
    return readings


def main() -> None:
    dev = open_device()

    app = QtWidgets.QApplication(sys.argv)
    win = pg.GraphicsLayoutWidget(title="MSP430FR2633 Heatmap")
    view = win.addViewBox()
    view.setAspectLocked(True)
    image = pg.ImageItem(np.zeros((8, 8), dtype=np.uint16))
    view.addItem(image)
    win.show()

    def update() -> None:
        data = np.array(read_scan(dev), dtype=np.uint16).reshape(8, 8)
        image.setImage(data, autoLevels=True)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # ~20 FPS

    try:
        app.exec()
    finally:
        timer.stop()
        dev.close()


if __name__ == "__main__":
    main()

