#include <Wire.h>

// I2C address of MSP430FR2633
const uint8_t FR2633_ADDR = 0x0A;

// Command to request a cycle packet from the FR2633
const uint8_t TL_CYCLE_PACKET_CMD = 0x01;

// Size of a full 4-element cycle packet from FR2633
const uint8_t CYCLE_PACKET_SIZE = 22;
const uint8_t CYCLE_PACKET_FALLBACK_SIZE = 10; // when only one element is returned

// Frame header for UART transmission
const char FRAME_HEADER[4] = {'C', 'A', '6', '4'}; // 'CA64'
const uint16_t PAYLOAD_LEN = 128; // 64 elements * 2 bytes

uint8_t seq = 0; // sequence counter for frames

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Request one cycle from the FR2633 and store Raw values
// readings: array of 64 uint16_t elements
void read_cycle(uint8_t cycle, uint16_t *readings) {
  Wire.beginTransmission(FR2633_ADDR);
  Wire.write(TL_CYCLE_PACKET_CMD);
  Wire.write(0x00); // Sensor ID
  Wire.write(cycle);
  // send without releasing the bus to allow repeated start
  Wire.endTransmission(false);

  uint8_t buf[CYCLE_PACKET_SIZE];
  uint8_t len = Wire.requestFrom(FR2633_ADDR, CYCLE_PACKET_SIZE);
  if (len != CYCLE_PACKET_SIZE) {
    len = Wire.requestFrom(FR2633_ADDR, CYCLE_PACKET_FALLBACK_SIZE);
  }

  // If we failed to get any bytes, skip
  if (len < 10) {
    return;
  }

  for (uint8_t k = 0; k < 4 && (6 + 4 * k + 3) < len; ++k) {
    uint8_t base = 6 + 4 * k;
    uint16_t raw = buf[base + 2] | ((uint16_t)buf[base + 3] << 8);
    uint8_t E = 8 * (cycle >> 1) + 2 * k + (cycle & 1);
    readings[E] = raw;
  }
}

// Collect all 64 raw values from the FR2633
void collect_scan(uint16_t *readings) {
  for (uint8_t i = 0; i < 64; ++i) {
    readings[i] = 0;
  }
  for (uint8_t cycle = 0; cycle < 16; ++cycle) {
    read_cycle(cycle, readings);
  }
}

// Send a CA64 frame containing the 64 raw readings
void send_frame(uint16_t *readings) {
  Serial.write((const uint8_t *)FRAME_HEADER, sizeof(FRAME_HEADER));
  Serial.write((uint8_t)(PAYLOAD_LEN & 0xFF));
  Serial.write((uint8_t)(PAYLOAD_LEN >> 8));
  Serial.write(seq++);

  // Prepare payload as little-endian bytes
  uint8_t payload[PAYLOAD_LEN];
  for (uint8_t i = 0; i < 64; ++i) {
    payload[2 * i] = readings[i] & 0xFF;
    payload[2 * i + 1] = readings[i] >> 8;
  }
  Serial.write(payload, PAYLOAD_LEN);

  // Compute CRC over [SEQ, LEN_lo, LEN_hi, PAYLOAD]
  uint8_t crc_data[1 + 2 + PAYLOAD_LEN];
  crc_data[0] = seq - 1; // seq already incremented
  crc_data[1] = PAYLOAD_LEN & 0xFF;
  crc_data[2] = PAYLOAD_LEN >> 8;
  memcpy(crc_data + 3, payload, PAYLOAD_LEN);
  uint16_t crc = crc16(crc_data, sizeof(crc_data));
  Serial.write(crc & 0xFF);
  Serial.write(crc >> 8);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      uint16_t readings[64];
      collect_scan(readings);
      send_frame(readings);
    }
  }
}

