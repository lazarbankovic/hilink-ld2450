#include "Ld2450.h"

#include "DelayTimer.h"

namespace hilink {
namespace {
int16_t twoByteToInt(char firstByte, char secondByte) {
  return (int16_t)((secondByte) << 8) | firstByte;
}

enum Ld2450Commands : uint8_t {
  GET_FW_VERSION = 0XA0,
  FACTORY_RESET = 0XA2,
  RESET = 0XA3,
  BLUETOOTH_SET = 0XA4,
  GET_MAC = 0XA5,
  SET_ZONES = 0XC2,
  GET_ZONES = 0XC1,
  ENTER_CONFIG = 0XFF,
  EXIT_CONFIG = 0XFE,
  SET_BAUD = A1,
  SET_SINGLETARGET = 0X80,
  SET_MULTITARGET = 0X90,
};

const static uint8_t RX_DATA_PREAMBLE[4] = {0xaa, 0xff, 0x03, 0x00};
const static uint8_t RX_DATA_POSTAMBLE[2] = {0x55, 0xcc};
const static uint8_t RX_CONFIG_PREAMBLE[4] = {0xfd, 0xfc, 0xfb, 0xfa};
const static uint8_t RX_CONFIG_POSTAMBLE[4] = {0x04, 0x03, 0x02, 0x01};

constexpr static int DELAY_MS = 1000;
}  // namespace

Ld2450::Ld2450(HardwareSerial& serial, SensorCallback&& callback)
    : m_serialPort(serial), m_callback(std::move(callback)) {}

bool Ld2450::reset() {
  sendCommand(Ld2450Commands::RESET, nullptr, 0);
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::setBluetooth(bool on) {
  uint8_t data[] = {on, 0x00};
  sendCommand(Ld2450Commands::BLUETOOTH_SET, data, sizeof(data));
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::setBaud(const Baud baud) {
  uint8_t data[] = {baud, 0x00};
  sendCommand(Ld2450Commands::SET_BAUD, data, sizeof(data));
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::setMultitarget() {
  sendCommand(Ld2450Commands::SET_MULTITARGET, nullptr, 0);
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::setSingletarget() {
  sendCommand(Ld2450Commands::SET_SINGLETARGET, nullptr, 0);
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::factoryReset() {
  sendCommand(Ld2450Commands::FACTORY_RESET, nullptr, 0);
  return getAckFrame(DELAY_MS, [](const DataPosition& pos) {});
}

bool Ld2450::getMac(uint8_t mac[6]) {
  uint8_t data[] = {0x01, 0x00};
  sendCommand(Ld2450Commands::GET_MAC, data, 2);
  return getAckFrame(DELAY_MS, [&](const DataPosition& pos) {
    if ((pos.end - pos.start) < 12) {
      return;  // sainty check
    }
    mac[0] = pos.start[6];
    mac[1] = pos.start[7];
    mac[2] = pos.start[8];
    mac[3] = pos.start[9];
    mac[4] = pos.start[10];
    mac[5] = pos.start[11];
  });
}

bool Ld2450::getFwVersion(String& version) {
  sendCommand(Ld2450Commands::GET_FW_VERSION, nullptr, 0);
  return getAckFrame(DELAY_MS, [&](const DataPosition& pos) {
    if ((pos.end - pos.start) < 11) {
      return;  // sainty check
    }
    char ver[15];
    sprintf(ver,
            "V%u.%02X.%02X%02X%02X%02X",
            pos.start[9],
            pos.start[8],
            pos.start[13],
            pos.start[12],
            pos.start[11],
            pos.start[10]);
    version = ver;
  });
}

bool Ld2450::setZones(const Zones& zones) {
  uint8_t data[] = {zones.type,
                    0x00,
                    (uint8_t)(zones.one.x1 & 0xff),
                    (uint8_t)(zones.one.x1 >> 8),
                    (uint8_t)(zones.one.y1 & 0xff),
                    (uint8_t)(zones.one.y1 >> 8),
                    (uint8_t)(zones.one.x2 & 0xff),
                    (uint8_t)(zones.one.x2 >> 8),
                    (uint8_t)(zones.one.y2 & 0xff),
                    (uint8_t)(zones.one.y2 >> 8),
                    (uint8_t)(zones.two.x1 & 0xff),
                    (uint8_t)(zones.two.x1 >> 8),
                    (uint8_t)(zones.two.y1 & 0xff),
                    (uint8_t)(zones.two.y1 >> 8),
                    (uint8_t)(zones.two.x2 & 0xff),
                    (uint8_t)(zones.two.x2 >> 8),
                    (uint8_t)(zones.two.y2 & 0xff),
                    (uint8_t)(zones.two.y2 >> 8),
                    (uint8_t)(zones.three.x1 & 0xff),
                    (uint8_t)(zones.three.x1 >> 8),
                    (uint8_t)(zones.three.y1 & 0xff),
                    (uint8_t)(zones.three.y1 >> 8),
                    (uint8_t)(zones.three.x2 & 0xff),
                    (uint8_t)(zones.three.x2 >> 8),
                    (uint8_t)(zones.three.y2 & 0xff),
                    (uint8_t)(zones.three.y2 >> 8)};
  sendCommand(Ld2450Commands::SET_ZONES, data, sizeof(data));
  return getAckFrame(DELAY_MS, [&](const DataPosition& pos) {});
}

bool Ld2450::getZones(Zones& zones) {
  sendCommand(Ld2450Commands::GET_ZONES, nullptr, 0);
  return getAckFrame(DELAY_MS, [&](const DataPosition& pos) {
    if ((pos.end - pos.start) < 32) {
      return;  // sainty check
    }
    zones.type = (ZoneType)(pos.start[6]);

    zones.one.x1 = twoByteToInt(pos.start[8], pos.start[9]);
    zones.one.y1 = twoByteToInt(pos.start[10], pos.start[11]);
    zones.one.x2 = twoByteToInt(pos.start[12], pos.start[13]);
    zones.one.y2 = twoByteToInt(pos.start[14], pos.start[15]);

    zones.two.x1 = twoByteToInt(pos.start[16], pos.start[17]);
    zones.two.y1 = twoByteToInt(pos.start[18], pos.start[19]);
    zones.two.x2 = twoByteToInt(pos.start[20], pos.start[21]);
    zones.two.y2 = twoByteToInt(pos.start[22], pos.start[23]);

    zones.three.x1 = twoByteToInt(pos.start[24], pos.start[25]);
    zones.three.y1 = twoByteToInt(pos.start[26], pos.start[27]);
    zones.three.x2 = twoByteToInt(pos.start[28], pos.start[29]);
    zones.three.y2 = twoByteToInt(pos.start[30], pos.start[31]);
  });
}

bool Ld2450::setConfigMode(bool on) {
  if (on) {
    uint8_t data[] = {on, 0x00};
    sendCommand(Ld2450Commands::ENTER_CONFIG, data, sizeof(data));
  } else {
    sendCommand(Ld2450Commands::EXIT_CONFIG, nullptr, 0);
  }
  return getAckFrame(DELAY_MS, [&](const DataPosition& pos) {});
}

void Ld2450::sendCommand(uint8_t command, uint8_t data[], uint8_t len) {
  uint8_t cmd[sizeof(RX_CONFIG_PREAMBLE) + sizeof(RX_CONFIG_POSTAMBLE) + len +
              4];  // 2 len + 2 command
  int8_t i = 0;
  memcpy(cmd, RX_CONFIG_PREAMBLE, sizeof(RX_CONFIG_PREAMBLE));
  i += sizeof(RX_CONFIG_PREAMBLE);
  cmd[i++] = len + 2;  // 2 bytes for command
  cmd[i++] = 0x00;
  cmd[i++] = command;
  cmd[i++] = 0x00;
  memcpy(cmd + i, data, len);
  memcpy(cmd + i + len, RX_CONFIG_POSTAMBLE, sizeof(RX_CONFIG_POSTAMBLE));
  m_serialPort.flush();
  m_serialPort.write(cmd, sizeof(cmd));
}

void Ld2450::init() { m_serialPort.begin(256000); }

void Ld2450::getDataFrame() {
  int available = m_serialPort.available();

  if (!available) {
    return;
  }

  uint8_t data[available];
  m_serialPort.readBytes(data, available);
  push(data, available);

  DataPosition position;

  /* Detect data frame in the buffer */
  if (!discover(position,
                RX_DATA_PREAMBLE,
                sizeof(RX_DATA_PREAMBLE),
                RX_DATA_POSTAMBLE,
                sizeof(RX_DATA_POSTAMBLE))) {
    return;  // Data frame not detected in the buffer
  }

  /* Extract target 1 data from the frame */
  auto sensorData = extractData(TARGET_1, position);
  if (sensorData.x != 0) {
    m_callback(TARGET_1, sensorData);
  }
  /* Extract target 2 data from the frame */
  sensorData = extractData(TARGET_2, position);
  if (sensorData.x != 0) {
    m_callback(TARGET_3, sensorData);
  }
  /* Extract target 3 data from the frame */
  sensorData = extractData(TARGET_3, position);
  if (sensorData.x != 0) {
    m_callback(TARGET_3, sensorData);
  }
  /* Remove the extracted frame from the buffer */
  shiftBufferLeft(position.end + sizeof(RX_DATA_POSTAMBLE));
}

void Ld2450::startData() { dataOnOff = true; }

void Ld2450::stopData() { dataOnOff = false; }

bool Ld2450::getAckFrame(
    uint16_t delayMs,
    std::function<void(const DataPosition& position)>&& callback) {
  DelayTimer timer{delayMs};
  DataPosition position;

  while (!timer.isExpired()) {
    int available = m_serialPort.available();

    if (!available) {
      continue;
    }

    uint8_t data[available];
    m_serialPort.readBytes(data, available);
    push(data, available);
    /* Detect config frame in the buffer */
    if (!discover(position,
                  RX_CONFIG_PREAMBLE,
                  sizeof(RX_CONFIG_PREAMBLE),
                  RX_CONFIG_POSTAMBLE,
                  sizeof(RX_CONFIG_POSTAMBLE))) {
      continue;
    }
    if (position.end - position.start < 5) {
      return false;  // sanity check
    }

    bool success = position.start[4] == 0x00;
    callback(position);
    shiftBufferLeft(position.end + sizeof(RX_CONFIG_POSTAMBLE));
    return success;
  }

  /* Timer expired */

  return false;
}

void Ld2450::tick() {
  if (dataOnOff) {
    getDataFrame();
  }
}

void Ld2450::shiftBufferLeft(uint8_t* newStart) {
  auto moveLeft = m_end - newStart;
  std::memcpy(m_buffer, newStart, moveLeft);
  m_end = m_buffer + (m_end - newStart);  // lower by rotated number of bytes
}

Ld2450::SensorData Ld2450::extractData(SensorTarget target,
                                       DataPosition position) {
  SensorData sensorData;

  int offset = (target * 8);  // each target 8 bytes
  sensorData.x =
      ((int16_t)position.start[offset + 1] << 8) | position.start[offset + 0];
  if (position.start[offset + 1] & 0x80)
    sensorData.x = -sensorData.x + 0x8000;
  sensorData.y = (position.start[offset + 3] << 8 | position.start[offset + 2]);
  if (sensorData.y != 0)
    sensorData.y -= 0x8000;
  sensorData.speed =
      position.start[offset + 5] << 8 | position.start[offset + 4];
  if (position.start[offset + 5] & 0x80)
    sensorData.speed = -sensorData.speed + 0x8000;
  sensorData.resolution =
      position.start[offset + 7] << 8 | position.start[offset + 6];

  return sensorData;
}

bool Ld2450::discover(DataPosition& position,
                      const uint8_t preamble[],
                      const uint8_t preambleSize,
                      const uint8_t postamble[],
                      const uint8_t postambleSize) {
  auto start = std::search(m_buffer, m_end, preamble, preamble + preambleSize);
  if (start == m_end) {
    return false;  // no preamble
  }

  auto end = std::search(m_buffer, m_end, postamble, postamble + postambleSize);
  if (end == m_end) {
    return false;  // no postamble
  }

  if (end < start) {
    return false;  // sanity check
  }

  position.start = start + sizeof(RX_DATA_PREAMBLE);
  position.end = end;
  return true;
}

void Ld2450::push(uint8_t* buf, uint8_t len) {
  if ((m_end + len) >=
      (m_buffer + MAX_BUF_SIZE))  // not enough space in the buffer
  {
    m_end = m_buffer;
    return;
  }

  memcpy(m_end, buf, len);
  m_end += len;
}
}  // namespace hilink