import sys
import struct
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

HEADER = b'CA64'
PAYLOAD_LEN = 128
FRAME_LEN = 4 + 2 + 1 + PAYLOAD_LEN + 2  # header + len + seq + payload + crc

# CRC16-CCITT (poly 0x1021, init 0xFFFF)
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class HeatmapViewer:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.img = pg.ImageItem(border='w')
        self.win = pg.GraphicsLayoutWidget(title="FR2633 Heatmap")
        self.view = self.win.addViewBox()
        self.view.addItem(self.img)
        self.array = [[0]*8 for _ in range(8)]
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def request_frame(self):
        self.ser.write(b'r')

    def read_frame(self) -> list:
        data = self.ser.read(FRAME_LEN)
        if len(data) != FRAME_LEN or not data.startswith(HEADER):
            return None
        payload = data[7:7+PAYLOAD_LEN]
        seq = data[6]
        length = struct.unpack_from('<H', data, 4)[0]
        if length != PAYLOAD_LEN:
            return None
        crc_recv = struct.unpack_from('<H', data, 7+PAYLOAD_LEN)[0]
        crc_calc = crc16(data[6:7+PAYLOAD_LEN])
        if crc_recv != crc_calc:
            return None
        values = list(struct.unpack('<64H', payload))
        return values

    def update(self):
        self.request_frame()
        values = self.read_frame()
        if values:
            arr = [values[i*8:(i+1)*8] for i in range(8)]
            self.img.setImage(arr, autoLevels=True)

    def run(self):
        self.win.show()
        QtCore.QTimer.singleShot(0, self.update)
        pg.exec()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python serial_heatmap.py <serial-port>')
        sys.exit(1)
    viewer = HeatmapViewer(sys.argv[1])
    viewer.run()
