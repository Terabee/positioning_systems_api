#ifndef RTLS_DRIVER_HPP
#define RTLS_DRIVER_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <map>
#include "serial_communication/iserial.hpp"
#include "logger/logger.hpp"

namespace terabee {

class RtlsDevice
{
public:
  enum class device_type
  {
    anchor = 0,
    tracker = 1
  };

  enum class tracker_msg_mode
  {
    msg_short = 0,
    msg_long = 1
  };

  struct anchor_data_t
  {
    uint8_t number;
    uint16_t id;
    int32_t pos_x;
    int32_t pos_y;
    int32_t pos_z;
    int32_t distance;
  };

  struct tracker_msg_t
  {
    uint32_t timestamp = 0;
    std::vector<anchor_data_t> anchors_data;
    std::array<int32_t, 3> tracker_position_xyz = {0, 0, 0};
    bool is_valid_position = false;
  };

  enum class cmd_header
  {
    device_anchor_priority = 0x00,
    device_tracker_priority = 0x01,
    device_label = 0x02,
    network_id = 0x03,
    update_time = 0x04,
    auto_anchor_positioning = 0x05,
    anchor_position = 0x06,
    anchor_initiator = 0x07,
    anchor_height_aap = 0x08,
    tracker_message_mode = 0x09,
    activate_leds = 0x10,
    reset_module = 0x99
  };

  struct config_t
  {
    device_type type;
    std::string fw_version;
    std::string serial_id;
    bool is_initiator;
    uint16_t priority;
    uint16_t label;
    uint16_t network_id;
    uint16_t update_time;
    bool is_auto_anchor_positioning;
    int32_t anchor_height;
    bool is_tracker_stream_mode;
    tracker_msg_mode tracker_message_mode;
    bool is_led_enabled;
    int32_t x;
    int32_t y;
    int32_t z;
  };

  using OnTrackerDataCallback = std::function<void(const tracker_msg_t&)>;

  static constexpr size_t TRACKER_STREAM_SHORT_LEN = 4;
  static constexpr size_t TRACKER_STREAM_LONG_1_ANCHOR_LEN =
      TRACKER_STREAM_SHORT_LEN + 6;
  static constexpr size_t TRACKER_STREAM_LONG_2_ANCHOR_LEN =
      TRACKER_STREAM_LONG_1_ANCHOR_LEN + 6;
  static constexpr size_t TRACKER_STREAM_LONG_3_ANCHOR_LEN =
      TRACKER_STREAM_LONG_2_ANCHOR_LEN + 6;
  static constexpr size_t TRACKER_STREAM_LONG_4_ANCHOR_LEN =
      TRACKER_STREAM_LONG_3_ANCHOR_LEN + 6;
  static constexpr size_t TRACKER_STREAM_MAX_ANCHOR_NUMBER = 4;

  static constexpr size_t CMD_ACK_LENGTH = 5;
  static constexpr size_t CMD_STD_LEN = 3;
  static constexpr size_t CMD_16B_DATA_LENGTH = 4;
  static constexpr size_t CMD_ANCHOR_POSITION_LENGTH = 14;
  static constexpr size_t CMD_ANCHOR_HEIGHT_AAP_LENGTH = 6;
  static const std::array<uint8_t, CMD_STD_LEN> auto_anchor_positioning_enable_;
  static const std::array<uint8_t, CMD_STD_LEN> auto_anchor_positioning_disable_;
  static const std::array<uint8_t, CMD_STD_LEN> anchor_initiator_enable_;
  static const std::array<uint8_t, CMD_STD_LEN> anchor_initiator_disable_;
  static const std::array<uint8_t, CMD_STD_LEN> led_enable_;
  static const std::array<uint8_t, CMD_STD_LEN> led_disable_;
  static const std::array<uint8_t, CMD_STD_LEN> restart_device_;
  static const std::array<uint8_t, CMD_STD_LEN> request_config_;
  static const std::array<uint8_t, CMD_STD_LEN> tracker_msg_short_;
  static const std::array<uint8_t, CMD_STD_LEN> tracker_msg_long_;
  static const std::array<uint8_t, CMD_STD_LEN> tracker_stream_enable_;
  static const std::array<uint8_t, CMD_STD_LEN> tracker_stream_disable_;

  RtlsDevice() = delete;
  explicit RtlsDevice(std::shared_ptr<serial_communication::ISerial> serialIf,
                      int cmd_send_interval_ms = 200);
  RtlsDevice(const RtlsDevice &other) = delete;
  RtlsDevice(RtlsDevice &&other) = delete;
  ~RtlsDevice();

  bool setDevice(device_type dev_type, uint16_t priority);
  bool setLabel(uint16_t label);
  bool setNetworkId(uint16_t network_id);
  bool setUpdateTime(uint16_t time_n100ms);
  bool setAnchorPosition(int32_t x, int32_t y, int32_t z);
  bool setAnchorHeightForAutoPositioning(int32_t height);
  bool enableAnchorInitiator();
  bool disableAnchorInitiator();
  bool enableAutoAnchorPositioning();
  bool disableAutoAnchorPositioning();
  bool enableLED();
  bool disableLED();
  bool restartDevice();
  bool requestConfig();
  bool setTrackerMessageShort();
  bool setTrackerMessageLong();
  bool enableTrackerStream();
  void disableTrackerStream();
  config_t getConfig() const { return readback_config_; }
  static std::vector<std::string> splitString(const std::string &input_line, char delimiter);

  bool registerOnDistanceDataCaptureCallback(OnTrackerDataCallback cb);
  bool startReadingStream();
  bool stopReadingStream();

private:
  utility::logger::Logger logger_;
  config_t readback_config_;
  std::chrono::milliseconds command_send_interval_ms_;

  OnTrackerDataCallback tracker_data_callback_;
  std::thread tracker_data_thread_;
  std::atomic_bool stop_;

  std::shared_ptr<serial_communication::ISerial> serial_port_;
  const float MM_TO_M_FACTOR = 0.001F;
  const float DEG_TO_RAD_FACTOR = M_PI/180.0;

  using ConfigOutputMap = std::map<std::string, std::string>;

  enum class device_params
  {
    type,
    fw_version,
    serial_id,
    is_initiator,
    priority,
    label,
    network_id,
    update_time,
    is_auto_anchor_positioning,
    anchor_height,
    is_tracker_stream_mode,
    tracker_message_mode,
    is_led_enabled,
    x,
    y,
    z,
    is_uart_ready
  };

  const std::map<device_params, std::string> PARAM_TO_STRING
  {
    { device_params::type, "Device Type" },
    { device_params::serial_id, "Serial ID" },
    { device_params::fw_version, "TERABEE UWB 3D RTLS Firmware Version" },
    { device_params::is_initiator, "Initiator Mode" },
    { device_params::priority, "Device Priority" },
    { device_params::label, "Device Label" },
    { device_params::network_id, "Network ID" },
    { device_params::update_time, "Network Update Time[ms]" },
    { device_params::is_auto_anchor_positioning, "Auto Anchor Positioning (AAP)" },
    { device_params::anchor_height, "Anchor Height for AAP [mm]" },
    { device_params::is_tracker_stream_mode, "Tracker Stream Mode" },
    { device_params::tracker_message_mode, "Tracker Message Mode" },
    { device_params::is_led_enabled, "LED Mode" },
    { device_params::x, "Device Position X [mm]" },
    { device_params::y, "Device Position Y [mm]" },
    { device_params::z, "Device Position Z [mm]" }
  };

  bool send16bDataCommand(cmd_header header, uint16_t data);
  bool sendRawCommand(const uint8_t cmd[], size_t length);
  bool readACK() const;

  bool readConfig();
  void updateConfigFromMap(const ConfigOutputMap &config_map);
  bool parseBool(std::string param);
  int32_t parseDecInt32(std::string coordinate);
  uint16_t parseHexUint16(std::string param);
  uint16_t parseDecUint16(std::string update_time);
  device_type parseDeviceType(std::string device_type_string);
  tracker_msg_mode parseTrackerMsgMode(std::string tracker_msg_mode);
  void readTrackerStream();
};

}  // namespace terabee

#endif // RTLS_DRIVER_HPP
