# MSP430FR2633 I2C Bridge

This repository contains an Arduino Uno sketch that bridges between a PC and a Texas Instruments MSP430FR2633 capacitive touch controller. The Arduino communicates with the FR2633 over I²C and streams raw sensor data back to the PC over UART.

## Firmware

- **fr2633_bridge.ino** – firmware for Arduino Uno
  - The PC sends the character `r` over the serial port to request one full scan.
  - The Arduino polls all 16 CapTIvate cycles from the MSP430 and sends a `CA64` frame containing 64 little-endian `uint16_t` raw values along with a CRC16 checksum.

## Python viewer

- **serial_heatmap.py** – simple PyQtGraph visualizer
  - Usage: `python serial_heatmap.py <serial-port>`
  - The script periodically sends `r` to the Arduino, parses the returned frame, and displays an 8×8 heatmap of the raw values.

## Building

1. Open `fr2633_bridge.ino` in the Arduino IDE.
2. Select **Arduino Uno** and the correct serial port.
3. Upload the sketch.

## Requirements

- Arduino Uno connected to MSP430FR2633 over I²C (400 kHz, address `0x0A`).
- Python 3 with `pyserial` and `pyqtgraph` installed for the heatmap viewer.
