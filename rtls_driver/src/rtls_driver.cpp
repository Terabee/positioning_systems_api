#include <iostream>
#include <algorithm>
#include <sstream>
#include "rtls_driver/rtls_driver.hpp"
#include "serial_communication/crc.hpp"

namespace terabee {

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::auto_anchor_positioning_enable_ = {0x05, 0x01, 0x46};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::auto_anchor_positioning_disable_{0x05, 0x00, 0x41};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::anchor_initiator_enable_{0x07, 0x01, 0x6C};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::anchor_initiator_disable_{0x07, 0x00, 0x6B};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::led_enable_{0x10, 0x01, 0x50};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::led_disable_{0x10, 0x00, 0x57};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::restart_device_{0x99, 0x99, 0x9A};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::request_config_{0x91, 0xDF, 0xE7};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::tracker_msg_short_{0x09, 0x00, 0xBD};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::tracker_msg_long_{0x09, 0x01, 0xBA};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::tracker_stream_enable_{0x11, 0x01, 0x45};

const std::array<uint8_t, RtlsDevice::CMD_STD_LEN>
  RtlsDevice::tracker_stream_disable_{0x11, 0x00, 0x42};

RtlsDevice::RtlsDevice(std::shared_ptr<serial_communication::ISerial> serialIf,
                       int cmd_send_interval_ms):
  logger_("RtlsDevice"),
  command_send_interval_ms_(cmd_send_interval_ms),
  tracker_data_thread_(),
  stop_(true),
  serial_port_(serialIf)
{
  logger_->debug("RtlsDevice::RtlsDevice(std::shared_ptr<serial_communication::ISerial>)");
}

RtlsDevice::~RtlsDevice()
{
  if (tracker_data_thread_.joinable()) {
    tracker_data_thread_.join();
  }
  logger_->debug("RtlsDevice::~RtlsDevice()");
}

bool RtlsDevice::setDevice(RtlsDevice::device_type dev_type, uint16_t priority)
{
  if (!send16bDataCommand(static_cast<cmd_header>(dev_type), priority))
  {
    logger_->error("Failed to set device type and priority!");
    return false;
  }
  readback_config_.type = dev_type;
  readback_config_.priority = priority;
  logger_->info("Device type set to: {}, priority: {}", static_cast<int>(dev_type), priority);
  return true;
}

bool RtlsDevice::setLabel(uint16_t label)
{
  if (!send16bDataCommand(cmd_header::device_label, label))
  {
    logger_->error("Failed to set device label!");
    return false;
  }

  readback_config_.label = label;
  logger_->info("Device label set to: 0x{0:x}", label);
  return true;
}

bool RtlsDevice::setNetworkId(uint16_t network_id)
{
  if (!send16bDataCommand(cmd_header::network_id, network_id))
  {
    logger_->error("Failed to set network ID!");
    return false;
  }

  readback_config_.network_id = network_id;
  logger_->info("Network ID set to: 0x{0:x}", network_id);
  return true;
}

/*
 * Sets update time with multiples of 100ms
 * E.g. time_n100ms = 3 gives update time of 300ms
 */
bool RtlsDevice::setUpdateTime(uint16_t time_n100ms)
{
  if (time_n100ms < 1 || time_n100ms > 600)
  {
    logger_->error("Update time parameter must fit in range <1;600>!");
    return false;
  }
  if (!send16bDataCommand(cmd_header::update_time, time_n100ms))
  {
    logger_->error("Failed to set update time!");
    return false;
  }

  readback_config_.update_time = time_n100ms;
  logger_->info("Update time set to: {}", time_n100ms*100);
  return true;
}

bool RtlsDevice::setAnchorPosition(int32_t x, int32_t y, int32_t z)
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  std::array<uint8_t, CMD_ANCHOR_POSITION_LENGTH> write_buffer = {
    static_cast<uint8_t>(cmd_header::anchor_position),
    static_cast<uint8_t>(x >> 24),
    static_cast<uint8_t>(x >> 16),
    static_cast<uint8_t>(x >> 8),
    static_cast<uint8_t>(x >> 0),
    static_cast<uint8_t>(y >> 24),
    static_cast<uint8_t>(y >> 16),
    static_cast<uint8_t>(y >> 8),
    static_cast<uint8_t>(y >> 0),
    static_cast<uint8_t>(z >> 24),
    static_cast<uint8_t>(z >> 16),
    static_cast<uint8_t>(z >> 8),
    static_cast<uint8_t>(z >> 0),
    0x0, // CRC8 placeholder
  };
  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendRawCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to set anchor position!");
    return false;
  }

  readback_config_.x = x;
  readback_config_.y = y;
  readback_config_.z = z;
  logger_->info("Anchor position set to: ({}, {}, {}) mm", x, y, z);
  return true;
}

bool RtlsDevice::setAnchorHeightForAutoPositioning(int32_t height)
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  std::array<uint8_t, CMD_ANCHOR_HEIGHT_AAP_LENGTH> write_buffer = {
    static_cast<uint8_t>(cmd_header::anchor_height_aap),
    static_cast<uint8_t>(height >> 24),
    static_cast<uint8_t>(height >> 16),
    static_cast<uint8_t>(height >> 8),
    static_cast<uint8_t>(height >> 0),
    0x0, // CRC8 placeholder
  };
  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  if (!sendRawCommand(write_buffer.data(), write_buffer.size()))
  {
    logger_->error("Failed to set anchor height!");
    return false;
  }

  readback_config_.anchor_height = height;
  logger_->info("Anchor height set to: {} mm", height);
  return true;
}

bool RtlsDevice::enableAnchorInitiator()
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  if (!sendRawCommand(anchor_initiator_enable_.data(),
                      anchor_initiator_enable_.size()))
  {
    logger_->error("Failed to enable anchor initiator!");
    return false;
  }

  readback_config_.is_initiator = true;
  logger_->info("Anchor initiator enabled");
  return true;
}

bool RtlsDevice::disableAnchorInitiator()
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  if (!sendRawCommand(anchor_initiator_disable_.data(),
                      anchor_initiator_disable_.size()))
  {
    logger_->error("Failed to disable anchor initiator!");
    return false;
  }

  readback_config_.is_initiator = false;
  logger_->info("Anchor initiator disabled");
  return true;
}

bool RtlsDevice::enableAutoAnchorPositioning()
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  if (!sendRawCommand(auto_anchor_positioning_enable_.data(),
                      auto_anchor_positioning_enable_.size()))
  {
    logger_->error("Failed to enable Auto Anchor Positioning!");
    return false;
  }

  readback_config_.is_auto_anchor_positioning = true;
  logger_->info("Auto Anchor Positioning enabled");
  return true;
}

bool RtlsDevice::disableAutoAnchorPositioning()
{
  if (readback_config_.type != device_type::anchor)
  {
    logger_->error("Device is not configured as anchor!");
    return false;
  }

  if (!sendRawCommand(auto_anchor_positioning_disable_.data(),
                      auto_anchor_positioning_disable_.size()))
  {
    logger_->error("Failed to disable Auto Anchor Positioning!");
    return false;
  }

  readback_config_.is_auto_anchor_positioning = false;
  logger_->info("Auto Anchor Positioning disabled");
  return true;
}

bool RtlsDevice::enableLED()
{
  if (!sendRawCommand(led_enable_.data(), led_enable_.size()))
  {
    logger_->error("Failed to enable LED!");
    return false;
  }

  readback_config_.is_led_enabled = true;
  logger_->info("LED enabled");
  return true;
}

bool RtlsDevice::disableLED()
{
  if (!sendRawCommand(led_disable_.data(), led_disable_.size()))
  {
    logger_->error("Failed to disable LED!");
    return false;
  }

  readback_config_.is_led_enabled = false;
  logger_->info("LED disabled");
  return true;
}

bool RtlsDevice::restartDevice()
{
  if (!sendRawCommand(restart_device_.data(), restart_device_.size()))
  {
    logger_->error("Failed to restart device!");
    return false;
  }

  logger_->info("Device restart triggered");
  return true;
}

bool RtlsDevice::requestConfig()
{
  if (serial_port_->write(request_config_.data(), request_config_.size()) <= 0)
  {
    logger_->error("Timeout or error while requesting device config!");
    return false;
  }

  if (!readConfig())
  {
    logger_->error("Parsing of device config failed!");
    return false;
  }

  logger_->info("Config received and parsed successfully");
  std::this_thread::sleep_for(std::chrono::milliseconds(command_send_interval_ms_));
  return true;
}

bool RtlsDevice::setTrackerMessageShort()
{
  if (readback_config_.type != device_type::tracker)
  {
    logger_->error("Device is not configured as tracker!");
    return false;
  }

  if (!sendRawCommand(tracker_msg_short_.data(), tracker_msg_short_.size()))
  {
    logger_->error("Failed to set tracker message type Short!");
    return false;
  }

  readback_config_.tracker_message_mode = tracker_msg_mode::msg_short;
  logger_->info("Tracker message type set to Short");
  return true;
}

bool RtlsDevice::setTrackerMessageLong()
{
  if (readback_config_.type != device_type::tracker)
  {
    logger_->error("Device is not configured as tracker!");
    return false;
  }

  if (!sendRawCommand(tracker_msg_long_.data(), tracker_msg_long_.size()))
  {
    logger_->error("Failed to set tracker message type Long!");
    return false;
  }

  readback_config_.tracker_message_mode = tracker_msg_mode::msg_long;
  logger_->info("Tracker message type set to Long");
  return true;
}

bool RtlsDevice::enableTrackerStream()
{
  serial_port_->flushInput();
  if (!sendRawCommand(tracker_stream_enable_.data(), tracker_stream_enable_.size()))
  {
    logger_->error("Failed to enable tracker stream!");
    return false;
  }

  readback_config_.is_tracker_stream_mode = true;
  logger_->info("Tracker stream enabled");
  return true;
}

void RtlsDevice::disableTrackerStream()
{
  if (serial_port_->write(tracker_stream_disable_.data(), tracker_stream_disable_.size()) <= 0)
  {
    logger_->error("Failed to disable tracker stream!");
    return;
  }

  readback_config_.is_tracker_stream_mode = false;
  logger_->info("Request to disable tracker stream sent");

  std::this_thread::sleep_for(std::chrono::milliseconds(command_send_interval_ms_));
  serial_port_->flushInput();
  return;
}

bool RtlsDevice::send16bDataCommand(cmd_header header, uint16_t data)
{
  std::array<uint8_t, CMD_16B_DATA_LENGTH> write_buffer = {
    static_cast<uint8_t>(header),
    static_cast<uint8_t>(data >> 8),
    static_cast<uint8_t>(data & 0xFF),
    0x0, // CRC8 placeholder
  };
  write_buffer.at(write_buffer.size() - 1) =
    crc::crc8(write_buffer.data(), write_buffer.size() - 1);

  logger_->debug("Sending 0x{0:x}, 0x{1:x}, 0x{2:x}, 0x{3:x}",
    static_cast<int>(write_buffer.at(0)),
    static_cast<int>(write_buffer.at(1)),
    static_cast<int>(write_buffer.at(2)),
    static_cast<int>(write_buffer.at(3))
  );

  return sendRawCommand(write_buffer.data(), write_buffer.size());
}

bool RtlsDevice::sendRawCommand(const uint8_t cmd[], size_t length)
{
  if (serial_port_->write(cmd, length) <= 0)
  {
    logger_->error("Timeout or error while writing to serial!");
    return false;
  }
  return readACK();
}

bool RtlsDevice::readACK() const
{
  bool success = true;
  std::array<uint8_t, CMD_ACK_LENGTH> ack_buffer = {0, 0, 0, 0, 0};
  size_t bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());
  if (bytes_read != ack_buffer.size() ||
      ack_buffer[0] != 0x99 ||
      ack_buffer[1] != 0x00 ||
      ack_buffer[2] != 0x5C ||
      ack_buffer[3] != '\r' ||
      ack_buffer[4] != '\n')
  {
    success = false;
    logger_->error("ACK Error. Received: 0x{0:x}, 0x{1:x}, 0x{2:x}, 0x{3:x}, 0x{4:x}",
      static_cast<int>(ack_buffer[0]),
      static_cast<int>(ack_buffer[1]),
      static_cast<int>(ack_buffer[2]),
      static_cast<int>(ack_buffer[3]),
      static_cast<int>(ack_buffer[4])
    );
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(command_send_interval_ms_));
  return success;
}

bool RtlsDevice::readConfig()
{
  ConfigOutputMap config_output_map =
  {
    { PARAM_TO_STRING.at(device_params::fw_version), "" },
    { PARAM_TO_STRING.at(device_params::serial_id), "" },
    { PARAM_TO_STRING.at(device_params::type), "" },
    { PARAM_TO_STRING.at(device_params::is_initiator), "" },
    { PARAM_TO_STRING.at(device_params::priority), "" },
    { PARAM_TO_STRING.at(device_params::label), "" },
    { PARAM_TO_STRING.at(device_params::network_id), "" },
    { PARAM_TO_STRING.at(device_params::update_time), "" },
    { PARAM_TO_STRING.at(device_params::is_auto_anchor_positioning), "" },
    { PARAM_TO_STRING.at(device_params::anchor_height), "" },
    { PARAM_TO_STRING.at(device_params::is_tracker_stream_mode), "" },
    { PARAM_TO_STRING.at(device_params::tracker_message_mode), "" },
    { PARAM_TO_STRING.at(device_params::is_led_enabled), "" },
    { PARAM_TO_STRING.at(device_params::x), "" },
    { PARAM_TO_STRING.at(device_params::y), "" },
    { PARAM_TO_STRING.at(device_params::z), "" },
  };

  std::string key_str;
  do
  {
    std::string line = serial_port_->readline();
    if (line.empty() || line == "CONFIG:END")
      break;

    size_t pos = line.find(":");
    key_str = line.substr(0, pos);

    auto element = config_output_map.find(key_str);
    if (element == config_output_map.end())
      continue;

    std::string value = line.substr(pos + 1);
    logger_->debug("Value: \"{}\" found for key: \"{}\"", value, key_str);

    element->second = value;
  } while (!key_str.empty());

  bool all_config_read = std::all_of(config_output_map.begin(), config_output_map.end(),
      [](const auto& conf)
        { return !conf.second.empty(); }
    );

  if (!all_config_read)
  {
    logger_->error("Did not read all config parameters!");
    return false;
  }

  updateConfigFromMap(config_output_map);

  return true;
}

void RtlsDevice::updateConfigFromMap(const ConfigOutputMap &config_map)
{
  readback_config_.fw_version = config_map.at(PARAM_TO_STRING.at(device_params::fw_version));
  readback_config_.serial_id = config_map.at(PARAM_TO_STRING.at(device_params::serial_id));
  readback_config_.type = parseDeviceType(config_map.at(PARAM_TO_STRING.at(device_params::type)));
  readback_config_.is_initiator = parseBool(config_map.at(PARAM_TO_STRING.at(device_params::is_initiator)));
  readback_config_.priority = parseDecUint16(config_map.at(PARAM_TO_STRING.at(device_params::priority)));
  readback_config_.label = parseHexUint16(config_map.at(PARAM_TO_STRING.at(device_params::label)));
  readback_config_.network_id = parseHexUint16(config_map.at(PARAM_TO_STRING.at(device_params::network_id)));
  readback_config_.update_time = parseDecUint16(config_map.at(PARAM_TO_STRING.at(device_params::update_time)));
  readback_config_.is_auto_anchor_positioning = parseBool(config_map.at(PARAM_TO_STRING.at(device_params::is_auto_anchor_positioning)));
  readback_config_.anchor_height = parseDecInt32(config_map.at(PARAM_TO_STRING.at(device_params::anchor_height)));
  readback_config_.is_tracker_stream_mode = parseBool(config_map.at(PARAM_TO_STRING.at(device_params::is_tracker_stream_mode)));
  readback_config_.tracker_message_mode = parseTrackerMsgMode(config_map.at(PARAM_TO_STRING.at(device_params::tracker_message_mode)));
  readback_config_.is_led_enabled = parseBool(config_map.at(PARAM_TO_STRING.at(device_params::is_led_enabled)));
  readback_config_.x = parseDecInt32(config_map.at(PARAM_TO_STRING.at(device_params::x)));
  readback_config_.y = parseDecInt32(config_map.at(PARAM_TO_STRING.at(device_params::y)));
  readback_config_.z = parseDecInt32(config_map.at(PARAM_TO_STRING.at(device_params::z)));
}

bool RtlsDevice::parseBool(std::string param)
{
  return param == "Enabled" ? true : false;
}

uint16_t RtlsDevice::parseHexUint16(std::string param)
{
  return static_cast<uint16_t>(std::strtoul(param.data(), nullptr, 16));
}

int32_t RtlsDevice::parseDecInt32(std::string coordinate)
{
  return std::stoi(coordinate);
}

uint16_t RtlsDevice::parseDecUint16(std::string update_time)
{
  return static_cast<uint16_t>(std::stoi(update_time));
}

RtlsDevice::device_type RtlsDevice::parseDeviceType(std::string device_type_string)
{
  if (device_type_string == "Anchor")
  {
    return device_type::anchor;
  }

  return device_type::tracker;
}

RtlsDevice::tracker_msg_mode RtlsDevice::parseTrackerMsgMode(std::string tracker_msg_mode)
{
  if (tracker_msg_mode == "Long")
  {
    return tracker_msg_mode::msg_long;
  }

  return tracker_msg_mode::msg_short;
}

std::vector<std::string> RtlsDevice::splitString(const std::string &input_line, char delimiter)
{
  std::vector<std::string> fields;
  std::string field;
  std::istringstream line(input_line);

  while (std::getline(line, field, delimiter))
  {
    fields.push_back(field);
  }
  return fields;
}

bool RtlsDevice::registerOnDistanceDataCaptureCallback(OnTrackerDataCallback cb)
{
  if (tracker_data_thread_.joinable()) {
    logger_->warn("Cannot change callback after device initialization");
    return false;
  }
  tracker_data_callback_ = cb;
  return true;
}

bool RtlsDevice::startReadingStream()
{
  if (readback_config_.type != device_type::tracker) {
    logger_->error("Cannot start reading tracker data output:"
                   " Device not configured as tracker!");
    return false;
  }

  if (tracker_data_thread_.joinable()) {
    logger_->error("Cannot start streaming. Thread already running!");
    return false;
  }

  stop_ = false;
  tracker_data_thread_ = std::thread(
    [this]() {
      logger_->debug("Stream reading thread started.");
      serial_port_->flushInput();
      while (!stop_) {
        readTrackerStream();
      }
      logger_->debug("Stream reading thread finished.");
    });

  return true;
}

bool RtlsDevice::stopReadingStream()
{
  stop_ = true;
  if (tracker_data_thread_.joinable()) {
    tracker_data_thread_.join();
  }
  serial_port_->flushInput();

  return !tracker_data_thread_.joinable();
}

void RtlsDevice::readTrackerStream()
{
  tracker_msg_t tracker_message;
  tracker_message.is_valid_position = true;
  std::string tracker_line = serial_port_->readline();
  tracker_line.erase(std::find(tracker_line.begin(), tracker_line.end(), '\0'),
                     tracker_line.end());
  logger_->debug("Tracker output: {}", tracker_line);
  std::vector<std::string> fields = splitString(tracker_line, ',');

  if (fields.size() != TRACKER_STREAM_SHORT_LEN &&
      fields.size() != TRACKER_STREAM_LONG_1_ANCHOR_LEN &&
      fields.size() != TRACKER_STREAM_LONG_2_ANCHOR_LEN &&
      fields.size() != TRACKER_STREAM_LONG_3_ANCHOR_LEN &&
      fields.size() != TRACKER_STREAM_LONG_4_ANCHOR_LEN)
  {
    logger_->error("Invalid frame of tracker output!");
    tracker_message.is_valid_position = false;
    return;
  }

  size_t cursor = 0;
  tracker_message.timestamp = static_cast<uint32_t>(std::stoi(fields.at(cursor++)));

  // Parse anchors positions
  for (size_t i = 0; i < TRACKER_STREAM_MAX_ANCHOR_NUMBER; i++)
  {
    if (fields.at(cursor).at(0) == 'D')
    {
      anchor_data_t anchor_data;
      anchor_data.number = static_cast<uint8_t>(fields.at(cursor).at(1) - '0');
      ++cursor;
      anchor_data.id = parseHexUint16(fields.at(cursor++));
      anchor_data.pos_x = std::stoi(fields.at(cursor++));
      anchor_data.pos_y = std::stoi(fields.at(cursor++));
      anchor_data.pos_z = std::stoi(fields.at(cursor++));
      anchor_data.distance = std::stoi(fields.at(cursor++));
      tracker_message.anchors_data.push_back(anchor_data);
    }
  }

  // Parse tracker position
  for (size_t i = 0; i < 3; i++)
  {
    std::string coordinate_str = fields.at(cursor + i);
    if (coordinate_str == "N/A")
    {
      tracker_message.tracker_position_xyz.at(i) = 0;
      tracker_message.is_valid_position = false;
    }
    else
    {
      tracker_message.tracker_position_xyz.at(i) =
        std::stoi(coordinate_str);
    }
  }
  if (tracker_data_callback_)
  {
    tracker_data_callback_(tracker_message);
  }
}

}  // namespace terabee
