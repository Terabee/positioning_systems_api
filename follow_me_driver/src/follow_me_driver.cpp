#include <iostream>
#include "follow_me_driver/follow_me_driver.hpp"
#include "serial_communication/crc.hpp"

namespace terabee {

const std::array<uint8_t, FollowMeMasterBeacon::CMD_PRINTOUT_LENGTH>
  FollowMeMasterBeacon::CMD_PRINTOUT_TEXT = {0x00, 0x11, 0x01, 0x45};

const std::array<uint8_t, FollowMeMasterBeacon::CMD_PRINTOUT_LENGTH>
  FollowMeMasterBeacon::CMD_PRINTOUT_BINARY = {0x00, 0x11, 0x02, 0x4C};

const std::array<uint8_t, FollowMeMasterBeacon::CMD_SPAN_LENGTH>
  FollowMeMasterBeacon::CMD_SPAN_AUTOCALIBRATE = {0x00, 0x22, 0x00, 0x00, 0x95};

const std::array<uint8_t, FollowMeMasterBeacon::CMD_TEST_LENGTH>
  FollowMeMasterBeacon::CMD_TEST = {0x00, 0x00, 0x00, 0x00};



FollowMeMasterBeacon::FollowMeMasterBeacon(std::shared_ptr<serial_communication::ISerial> serialIf):
  logger_("FollowMeMasterBeacon"),
  serial_port_(serialIf)
{
}

bool FollowMeMasterBeacon::process(PolarPoint2D &point)
{
  if (binary_mode_)
  {
    return processFrameBinary(point);
  }
  else
  {
    return processFrameText(point);
  }
}

bool FollowMeMasterBeacon::printoutModeText()
{
  if (!sendCommand(CMD_PRINTOUT_TEXT.data(), CMD_PRINTOUT_TEXT.size()))
  {
    logger_->error("Failed to set output mode: Text!");
    return false;
  }

  binary_mode_ = false;
  logger_->info("Output mode: Text");
  return true;
}

bool FollowMeMasterBeacon::printoutModeBin()
{
  if (!sendCommand(CMD_PRINTOUT_BINARY.data(), CMD_PRINTOUT_BINARY.size()))
  {
    logger_->error("Failed to set output mode: Binary!");
    return false;
  }

  binary_mode_ = true;
  logger_->info("Output mode: Binary");
  return true;
}

bool FollowMeMasterBeacon::spanAutoCalibrate() const
{
  if (!sendCommand(CMD_SPAN_AUTOCALIBRATE.data(), CMD_SPAN_AUTOCALIBRATE.size()))
  {
    logger_->error("Failed to initialize span autocalibration!");
    return false;
  }
  logger_->info("Span autocalibration executed");
  return true;
}

bool FollowMeMasterBeacon::swapBeacons(bool is_swapped) const
{
  std::array<uint8_t, CMD_SWAP_BEACONS_LENGTH> write_buffer =
    { 0x00, 0x31, is_swapped, 0x00 };

  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to swap beacons!");
    return false;
  }
  logger_->info("Beacons swapped: {}", static_cast<int>(is_swapped));
  return true;
}

bool FollowMeMasterBeacon::setBeaconsSpan(uint16_t span_mm) const
{
  uint8_t span_mm_hi = span_mm >> 8;
  uint8_t span_mm_lo = span_mm & 0x00FF;

  std::array<uint8_t, CMD_SPAN_LENGTH> write_buffer =
    { 0x00, 0x22, span_mm_hi, span_mm_lo, 0x00 };

  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to set beacons span!");
    return false;
  }
  logger_->info("Beacons span: {}", static_cast<int>(span_mm));
  return true;
}

bool FollowMeMasterBeacon::setEMAWindow(uint8_t ema_window) const
{
  std::array<uint8_t, CMD_EMA_WINDOW_LENGTH> write_buffer =
    {0x00, 0x52, 0x01, ema_window, 0x00};

  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to set EMA window!");
    return false;
  }
  logger_->info("EMA window: {}", static_cast<int>(ema_window));
  return true;
}

bool FollowMeMasterBeacon::setRS485_Parameters(uint32_t baudrate, rs485_parity parity) const
{
  std::array<uint8_t, CMD_RS485_PARAMS_LENGTH> write_buffer =
    { 0x00, 0x55, 0x02,
      static_cast<uint8_t>(baudrate >> 16),
      static_cast<uint8_t>(baudrate >> 8),
      static_cast<uint8_t>(baudrate >> 0),
      static_cast<uint8_t>(parity),
      0x00
    };

  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to set RS485 parameters!");
    return false;
  }

  logger_->info("RS485 parameters set");
  return true;
}

bool FollowMeMasterBeacon::setRS485_SlaveId(uint8_t slave_id) const
{
  std::array<uint8_t, CMD_MODBUS_SLAVE_ID_LENGTH> write_buffer_slave_id =
    { 0x00, 0x52, 0x03, slave_id, 0x00 };

  write_buffer_slave_id.at(write_buffer_slave_id.size() - 1) =
    crc::crc8(write_buffer_slave_id.data(), write_buffer_slave_id.size() - 1);

  if (!sendCommand(write_buffer_slave_id.data(), write_buffer_slave_id.size()))
  {
    logger_->error("Failed to set RS485 slave id!");
    return false;
  }

  logger_->info("RS485 slave id set");
  return true;
}

bool FollowMeMasterBeacon::processFrameBinary(PolarPoint2D &point)
{
  size_t bytes_read = serial_port_->read(read_buffer_.data(), read_buffer_.size());

  if (bytes_read <= 0)
  {
    return false;
  }

  if (bytes_read != read_buffer_.size() || read_buffer_[0] != 'F' || read_buffer_[1] != 'M')
  {
    logger_->error("Frame incorrect: {} bytes; Header: {0:x}{1:x}!", bytes_read, read_buffer_[0], read_buffer_[1]);
    return false;
  }

  if (crc::crc8(read_buffer_.data(), read_buffer_.size() - 1) !=
      read_buffer_.back())
  {
    logger_->error("CRC incorrect!");
    return false;
  }

  point.distance = MM_TO_M_FACTOR*static_cast<float>((read_buffer_[2] << 8) + read_buffer_[3]);
  auto heading_read = static_cast<int8_t>(read_buffer_[4]);
  point.heading = DEG_TO_RAD_FACTOR*static_cast<float>(heading_read);

  return true;
}

bool FollowMeMasterBeacon::processFrameText(PolarPoint2D &point) const
{
  std::string data = serial_port_->readline(SERIAL_READ_TEXT_BUFFER_SIZE, '\n');
  if (data.empty())
  {
    return false;
  }

  size_t distance_pos = data.find('\t') + 1;
  size_t heading_pos = data.find('\t', distance_pos) + 1;

  if (data.substr(0, distance_pos) != "FM\t")
  {
    return false;
  }

  point.distance = MM_TO_M_FACTOR*std::stof(data.substr(distance_pos, heading_pos));
  point.heading = DEG_TO_RAD_FACTOR*std::stof(data.substr(heading_pos));

  return true;
}

std::string FollowMeMasterBeacon::retrieveDeviceData() const
{
  if (!writeCommand(CMD_TEST.data(), CMD_TEST.size()))
  {
    logger_->error("Failed to retrieve device information!");
    return "";
  }
  uint8_t eol;
  serial_port_->read(&eol, 1);
  return serial_port_->readline(SERIAL_READ_TEST_CMD_BUFFER_SIZE);
}

bool FollowMeMasterBeacon::sendCommand(const uint8_t cmd[], size_t length) const
{
  if (!writeCommand(cmd, length))
  {
    return false;
  }
  return readACK(cmd);
}

bool FollowMeMasterBeacon::writeCommand(const uint8_t cmd[], size_t length) const
{
  if(!serial_port_->write(cmd, length))
  {
    logger_->error("Timeout or error while writing to serial!");
    return false;
  }
  return true;
}

bool FollowMeMasterBeacon::readACK(const uint8_t cmd[]) const
{
  std::array<uint8_t, CMD_ACK_LENGTH> ack_buffer = {0, 0, 0, 0};
  size_t bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());
  if (bytes_read == ack_buffer.size() &&
      ack_buffer[0] == cmd[0] &&
      ack_buffer[1] == (cmd[1] >> 4) &&
      ack_buffer[2] == 0x00 &&
      ack_buffer[3] == crc::crc8(ack_buffer.data(), ack_buffer.size() - 1))
  {
    return true;
  }

  logger_->error("ACK error. Received: 0x{0:x}, 0x{1:x}, 0x{2:x}, 0x{3:x}",
    static_cast<int>(ack_buffer[0]),
    static_cast<int>(ack_buffer[1]),
    static_cast<int>(ack_buffer[2]),
    static_cast<int>(ack_buffer[3])
  );

  return false;
}

FollowMeRemoteControl::FollowMeRemoteControl(std::shared_ptr<serial_communication::ISerial> serialIf):
  logger_("FollowMeRemoteControl"),
  serial_port_(serialIf)
{
}

bool FollowMeRemoteControl::setButtonMode(button_mode mode) const
{
  std::array<uint8_t, CMD_BUTTON_MODE_LENGTH> remote_write = { 0x08, 0x00, 0x00 };
  bool success = false;

  switch (mode)
  {
  case button_mode::toggle:
    remote_write.at(1) = 0x01;
    remote_write.at(2) = 0xAF;
    success = writeCommandCheckACK(remote_write.data(), remote_write.size());
    break;
  case button_mode::hold:
    remote_write.at(1) = 0x00;
    remote_write.at(2) = 0xA8;
    success = writeCommandCheckACK(remote_write.data(), remote_write.size());
    break;
  default:
    logger_->error("Unsupported remote control button mode!");
    success = false;
    break;
  }

  if (success)
  {
    logger_->info("Button mode changed to {}", (mode == button_mode::toggle ? "toggle" : "hold"));
  }
  return success;
}

bool FollowMeRemoteControl::setBuzzer(bool enabled) const
{
  std::array<uint8_t, CMD_BUZZER_MODE_LENGTH> remote_write = { 0x09, 0x00, 0x00 };
  if (enabled)
  {
    remote_write.at(1) = 0x01;
    remote_write.at(2) = 0xBA;
  }
  else
  {
    remote_write.at(1) = 0x00;
    remote_write.at(2) = 0xBD;
  }

  bool success = writeCommandCheckACK(remote_write.data(), remote_write.size());

  if (success)
  {
    logger_->info("Buzzer {}", (enabled ? "activated" : "deactivated"));
  }
  return success;
}

bool FollowMeRemoteControl::retrieveRemoteParameters(FollowMeRemoteControl::button_mode &mode, bool &buzzer) const
{
  std::array<uint8_t, CMD_GET_PARAMS_LENGTH> remote_write = { 0x04, 0x00, 0x54 };
  std::array<uint8_t, CMD_GET_PARAMS_LENGTH> response_buffer = {};

  if (!serial_port_->write(remote_write.data(), remote_write.size()))
  {
    logger_->error("Timeout or error while writing to serial!");
    return false;
  }

  size_t bytes_read = serial_port_->read(response_buffer.data(), response_buffer.size());
  if (bytes_read == response_buffer.size() &&
      response_buffer.at(0) == 0x40 &&
      response_buffer.at(response_buffer.size() - 1) ==
        crc::crc8(response_buffer.data(), response_buffer.size() - 1))
  {
    buzzer = response_buffer.at(1) & 0x0F;
    mode = static_cast<FollowMeRemoteControl::button_mode>(response_buffer.at(1) >> 4);
    return true;
  }
  return false;
}

bool FollowMeRemoteControl::writeCommandCheckACK(const uint8_t frame[], size_t size) const
{
  if (!serial_port_->write(frame, size))
  {
    logger_->error("Timeout or error while writing to serial!");
    return false;
  }

  if (!readACK())
  {
    return false;
  }

  return true;
}

bool FollowMeRemoteControl::readACK() const
{
  std::array<uint8_t, CMD_ACK_LENGTH> ack_buffer;
  size_t bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());

  if (bytes_read < ack_buffer.size())
  {
    logger_->error("ACK Error: Not enough bytes read!");
    return false;
  }

  for (uint8_t el : ack_buffer)
  {
    if (el != 0x00)
    {
      logger_->error("ACK Error: NACK!");
      return false;
    }
  }

  return true;
}

}
