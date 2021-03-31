#ifndef FOLLOW_ME_DRIVER_H
#define FOLLOW_ME_DRIVER_H

#include <memory>
#include <cmath>
#include "serial_communication/iserial.hpp"
#include "logger/logger.hpp"

namespace terabee {

struct PolarPoint2D
{
  float distance;
  float heading;
};

class FollowMeMasterBeacon
{
public:
  enum class printout_mode
  {
    text = 1,
    binary = 2
  };

  enum class rs485_parity
  {
    rs485_parity_none = 0,
    rs485_parity_odd = 1,
    rs485_parity_even = 2
  };

  FollowMeMasterBeacon() = delete;
  explicit FollowMeMasterBeacon(std::shared_ptr<serial_communication::ISerial> serialIf);
  FollowMeMasterBeacon(const FollowMeMasterBeacon &other) = delete;
  FollowMeMasterBeacon(FollowMeMasterBeacon &&other) = delete;
  ~FollowMeMasterBeacon() = default;

  bool process(PolarPoint2D &point);

  bool printoutModeText();
  bool printoutModeBin();
  bool spanAutoCalibrate() const;
  bool swapBeacons(bool is_swapped) const;
  bool setBeaconsSpan(uint16_t span_mm) const;
  bool setEMAWindow(uint8_t ema_window) const;
  bool setRS485_Parameters(uint32_t baudrate, rs485_parity parity) const;
  bool setRS485_SlaveId(uint8_t slave_id) const;
  std::string retrieveDeviceData() const;

  static uint8_t calculateCommandLength(const uint8_t frame[]) { return (frame[1] & 0x0F) + 3; }

private:
  utility::logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  const float MM_TO_M_FACTOR = 0.001F;
  const float DEG_TO_RAD_FACTOR = M_PI/180.0;
  bool binary_mode_ = true;

  static constexpr size_t CMD_ACK_LENGTH = 4;
  static constexpr size_t CMD_PRINTOUT_LENGTH = 4;
  static constexpr size_t CMD_SPAN_LENGTH = 5;
  static constexpr size_t CMD_MODBUS_SLAVE_ID_LENGTH = 5;
  static constexpr size_t CMD_EMA_WINDOW_LENGTH = 5;
  static constexpr size_t CMD_TEST_LENGTH = 4;
  static constexpr size_t CMD_SWAP_BEACONS_LENGTH = 4;
  static constexpr size_t CMD_RS485_PARAMS_LENGTH = 8;

  static const std::array<uint8_t, CMD_PRINTOUT_LENGTH> CMD_PRINTOUT_TEXT;
  static const std::array<uint8_t, CMD_PRINTOUT_LENGTH> CMD_PRINTOUT_BINARY;
  static const std::array<uint8_t, CMD_SPAN_LENGTH> CMD_SPAN_AUTOCALIBRATE;
  static const std::array<uint8_t, CMD_TEST_LENGTH> CMD_TEST;

  static constexpr size_t SERIAL_READ_TEST_CMD_BUFFER_SIZE = 120;
  static constexpr size_t SERIAL_READ_TEXT_BUFFER_SIZE = 20;
  static constexpr size_t BINARY_FRAME_BUFFER_SIZE = 6;
  std::array<uint8_t, BINARY_FRAME_BUFFER_SIZE> read_buffer_;

  bool processFrameBinary(PolarPoint2D &point);
  bool processFrameText(PolarPoint2D &point) const;

  bool sendCommand(const uint8_t cmd[], size_t length) const;
  bool writeCommand(const uint8_t cmd[], size_t length) const;
  bool readACK(const uint8_t cmd[]) const;
};

class FollowMeRemoteControl
{
public:
  enum class button_mode
  {
    hold = 0,
    toggle = 1
  };

  FollowMeRemoteControl() = delete;
  explicit FollowMeRemoteControl(std::shared_ptr<serial_communication::ISerial> serialIf);
  FollowMeRemoteControl(const FollowMeRemoteControl &other) = delete;
  FollowMeRemoteControl(FollowMeRemoteControl &&other) = delete;
  ~FollowMeRemoteControl() = default;

  bool setButtonMode(button_mode mode) const;
  bool setBuzzer(bool enabled) const;
  bool retrieveRemoteParameters(button_mode &mode, bool &buzzer) const;

private:
  utility::logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;

  static constexpr size_t CMD_ACK_LENGTH = 6;
  static constexpr size_t CMD_BUTTON_MODE_LENGTH = 3;
  static constexpr size_t CMD_BUZZER_MODE_LENGTH = 3;
  static constexpr size_t CMD_GET_PARAMS_LENGTH = 3;

  bool writeCommandCheckACK(const uint8_t frame[], size_t size) const;
  bool readACK() const;
};

}  // namespace terabee

#endif // FOLLOW_ME_DRIVER_H
