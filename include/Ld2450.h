/*****************************************************************************
 * @file     <Ld2450>.h
 * @brief    Hilink ld2450 serial driver
 * @version  V1.0.0
 * @author   Lazar Bankovic
 *****************************************************************************/

#pragma once

#include <Arduino.h>

#include <cstring>

namespace hilink {

class Ld2450 {
 public:
  enum SensorTarget { TARGET_1 = 0, TARGET_2 = 1, TARGET_3 = 2 };
  struct SensorData {
    int x;
    int y;
    int speed;
    int resolution;
  };
  struct Zone {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
  };
  enum ZoneType : uint8_t { CLOSE_DETECTION, AREA_DETECTION, AREA_FILTERING };
  struct Zones {
    ZoneType type;
    Zone one;
    Zone two;
    Zone three;
  };
  enum Baud : uint8_t {
    BAUD_9600 = 0x01,
    BAUD_19200 = 0x02,
    BAUD_38400 = 0x03,
    BAUD_57600 = 0x04,
    BAUD_115200 = 0x05,
    BAUD_230400 = 0x06,
    BAUD_256000 = 0x07,
    BAUD_460800 = 0x08
  };

  using SensorCallback =
      std::function<void(SensorTarget target, const SensorData& data)>;

  Ld2450(HardwareSerial& serial, SensorCallback&& callback);

  /**
   * @brief Initialize the module
   *
   * Call this once in your setup()
   */
  void init();

  /**
   * @brief Turn bluetooth on / off
   *
   * Bluetooth state is preserver across reboots.
   *
   * @param on[in] (true - on, off - false)
   * @returns command successful
   */
  bool setBluetooth(bool on);

  /**
   * @brief Gets firmware version string
   *
   * @param[out] version string
   * @returns command successful
   */
  bool getFwVersion(String& version);

  /**
   * @brief Gets mac address of the bluetooth module
   *
   * @param[out] mac 6-byte mac address
   * @returns command successful
   */
  bool getMac(uint8_t mac[6]);

  /**
   * @brief Sets the serial communication baud
   *
   * @param[out] baud
   * @returns command successful
   */
  bool setBaud(const Baud baud);

  /**
   * @brief Does a factory reset to the radar module
   *
   * This command is used to restore all configuration values to factory
   * values, and the configurationvalues take effect after rebooting the module.
   *
   * @returns command successful
   */
  bool factoryReset();

  /**
   * @brief Set to single-target tracking
   *
   * @returns command successful
   */
  bool setSingletarget();

  /**
   * @brief Set to multi-target tracking
   *
   * @returns command successful
   */
  bool setMultitarget();

  /**
   * @brief Queries the current zone filtering configuration of the module
   *
   * Each zone is represented with 2 xy coordinates, top-left (x1,y1) and bottom
   * right (x2,y2).
   * Unit is millimeter, and coordinate system is split into two parts on x
   * axis.
   * |(-5500,7000)
   * |
   * |
   * |
   * |
   * |
   * |_ _ _ _ _ (0,0) _ _ _ _ _(5500,0)
   *
   * zones.type has 3 values:
   * CLOSE_DETECTION - disable region filtering
   * AREA_DETECTION - detect only the set region
   * AREA_FILTERING - do not detect the set area
   *
   * @param[out] zones zone coordinates
   * @returns command successful
   */
  bool getZones(Zones& zones);

  /**
   * @brief Sets the zone coordinates
   *
   * This command is used to set the region filtering configuration of the
   * module, the configurationvaluewill not be lost when power down, and it
   * takes effect immediately after setup
   *
   * @param[in] zones zone coordinates
   * @returns command successful
   */
  bool setZones(const Zones& zones);

  /**
   * @brief Resets the radar module
   *
   * @param[in] zones zone coordinates
   * @returns command successful
   */
  bool reset();

  /**
   * @brief Enters/exits the configuration mode
   *
   * Each of the commands can be executad only while module is in config mode.
   * If not in config mode, call to any config command will fail.
   * When the config is done, exit the config mode so the detection data can
   * start coming in.
   *
   * @param[in] on true enters, false exits
   * @returns command successful
   */
  bool setConfigMode(bool on);

  /**
   * @brief Start parsing detection data
   *
   * After this call to this method detection data will be parsed and on any
   * successful parse the callback registered through the constructor will be
   * called.
   * In order for this to work, pariodic call to tick() method is necessary.
   *
   */
  void startData();

  /**
   * @brief Stops receiving detection data
   */

  void stopData();

  /**
   * @brief Call this method periodically to get the results
   *
   * Call this in your loop() function as often as possible.
   *
   */

  void tick();

 private:
  struct DataPosition {
    uint8_t* start;
    uint8_t* end;
  };

  void shiftBufferLeft(uint8_t* newStart);

  void sendCommand(uint8_t command, uint8_t data[], uint8_t len);

  SensorData extractData(SensorTarget target, DataPosition position);

  bool discover(DataPosition& position,
                const uint8_t preamble[],
                const uint8_t preambleSize,
                const uint8_t postamble[],
                const uint8_t postambleSize);

  void push(uint8_t* buf, uint8_t len);

  void getDataFrame();

  bool getAckFrame(
      uint16_t delayMs,
      std::function<void(const DataPosition& position)>&& callback);

  constexpr static uint8_t MAX_BUF_SIZE = 150;
  HardwareSerial& m_serialPort;
  uint8_t m_buffer[MAX_BUF_SIZE];
  uint8_t* m_end = m_buffer;  // Buffer fill indicator
  SensorCallback m_callback;  // Will be called when the data is detected
  bool dataOnOff = false;
};
}  // namespace hilink