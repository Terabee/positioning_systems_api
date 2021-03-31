#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <future>
#include <cstring>
#include "rtls_driver/rtls_driver.hpp"
#include "serial_communication/crc.hpp"

#include "MockISerial.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;

namespace terabee {

class RtlsDriverFixture : public testing::Test
{
protected:
  RtlsDriverFixture():
    serialInterface_(std::make_shared<serial_communication::MockISerial>()),
    read_tracker_stream_(),
    m_(),
    read_times_(0)
  {
    ON_CALL(*serialInterface_, open()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, isOpen()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, available()).WillByDefault(Return(69));
    ON_CALL(*serialInterface_, write(_, _))
        .WillByDefault(Invoke(
           [this](const uint8_t* data, size_t size) {
             data_sent_to_write.assign(data, data + size);
             return size;
           }));
    ON_CALL(*serialInterface_, read(_, _))
        .WillByDefault(Invoke(
           [this](uint8_t* data, size_t size) {
             std::memcpy(data, ackDataBin_, 5);
             return size;
           }));
    ON_CALL(*serialInterface_, readline(_, _))
        .WillByDefault(Invoke(
             [this](size_t, char) {
               read_times_++;
               read_tracker_stream_.notify_all();
               return tracker_output_data_;
           }));

    rtls_.reset(new RtlsDevice(serialInterface_, 0));

    rtls_->registerOnDistanceDataCaptureCallback(
      [this](const terabee::RtlsDevice::tracker_msg_t& tracker_output)
      {
        std::cout << "T:" << tracker_output.timestamp << "\t";
        std::cout << "Tracker position: (" <<
          tracker_output.tracker_position_xyz.at(0) << ", " <<
          tracker_output.tracker_position_xyz.at(1) << ", " <<
          tracker_output.tracker_position_xyz.at(2) <<
        ")" << std::endl;
        tracker_msg_ = tracker_output;
      });
  }

  bool waitForTrackerData()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_tracker_stream_.wait_for(l, std::chrono::milliseconds(500),
      [this]()
      {
        return read_times_.load() > 2;
      });
  }

  virtual ~RtlsDriverFixture();

  std::unique_ptr<RtlsDevice> rtls_;
  std::shared_ptr<serial_communication::MockISerial> serialInterface_;

  std::vector<uint8_t> data_sent_to_write;
  uint8_t ackDataBin_[5] = { 0x99, 0x00, 0x5C, '\r', '\n' };
  uint8_t nackDataBin_[5] = { 0x99, 0xFF, 0xAF, '\r', '\n' };

  std::condition_variable read_tracker_stream_;
  std::string tracker_output_data_;
  terabee::RtlsDevice::tracker_msg_t tracker_msg_;
  std::mutex m_;
  std::atomic_int read_times_;
public:

};

RtlsDriverFixture::~RtlsDriverFixture() {
  rtls_.reset();
}

TEST_F(RtlsDriverFixture, validParameters)
{
  EXPECT_TRUE(rtls_->setLabel(0xFEED));
  EXPECT_EQ(rtls_->getConfig().label, 0xFEED);
  EXPECT_TRUE(rtls_->setNetworkId(0xFACE));
  EXPECT_EQ(rtls_->getConfig().network_id, 0xFACE);
  EXPECT_TRUE(rtls_->setUpdateTime(1));
  EXPECT_EQ(rtls_->getConfig().update_time, 1);
  EXPECT_TRUE(rtls_->setUpdateTime(600));
  EXPECT_EQ(rtls_->getConfig().update_time, 600);
  EXPECT_TRUE(rtls_->enableLED());
  EXPECT_EQ(rtls_->getConfig().is_led_enabled, true);
  EXPECT_TRUE(rtls_->disableLED());
  EXPECT_EQ(rtls_->getConfig().is_led_enabled, false);

  EXPECT_TRUE(rtls_->setDevice(RtlsDevice::device_type::anchor, 4));
  EXPECT_EQ(rtls_->getConfig().type, RtlsDevice::device_type::anchor);
  EXPECT_EQ(rtls_->getConfig().priority, 4);
  EXPECT_TRUE(rtls_->setAnchorPosition(1000, 200, 2000));
  EXPECT_EQ(rtls_->getConfig().x, 1000);
  EXPECT_EQ(rtls_->getConfig().y, 200);
  EXPECT_EQ(rtls_->getConfig().z, 2000);
  EXPECT_TRUE(rtls_->setAnchorHeightForAutoPositioning(1000));
  EXPECT_EQ(rtls_->getConfig().anchor_height, 1000);
  EXPECT_TRUE(rtls_->enableAnchorInitiator());
  EXPECT_EQ(rtls_->getConfig().is_initiator, true);
  EXPECT_TRUE(rtls_->disableAnchorInitiator());
  EXPECT_EQ(rtls_->getConfig().is_initiator, false);
  EXPECT_TRUE(rtls_->enableAutoAnchorPositioning());
  EXPECT_EQ(rtls_->getConfig().is_auto_anchor_positioning, true);
  EXPECT_TRUE(rtls_->disableAutoAnchorPositioning());
  EXPECT_EQ(rtls_->getConfig().is_auto_anchor_positioning, false);

  EXPECT_TRUE(rtls_->setDevice(RtlsDevice::device_type::tracker, 7));
  EXPECT_EQ(rtls_->getConfig().priority, 7);
  EXPECT_EQ(rtls_->getConfig().type, RtlsDevice::device_type::tracker);
  EXPECT_TRUE(rtls_->setTrackerMessageShort());
  EXPECT_EQ(rtls_->getConfig().tracker_message_mode, RtlsDevice::tracker_msg_mode::msg_short);
  EXPECT_TRUE(rtls_->setTrackerMessageLong());
  EXPECT_EQ(rtls_->getConfig().tracker_message_mode, RtlsDevice::tracker_msg_mode::msg_long);
  EXPECT_TRUE(rtls_->enableTrackerStream());
  EXPECT_EQ(rtls_->getConfig().is_tracker_stream_mode, true);
  rtls_->disableTrackerStream();
  EXPECT_EQ(rtls_->getConfig().is_tracker_stream_mode, false);

  EXPECT_TRUE(rtls_->restartDevice());

  std::vector<uint8_t> expected_data_sent = {0x91, 0xDF, 0xE7};
  rtls_->requestConfig();
  EXPECT_EQ(data_sent_to_write, expected_data_sent);
}

TEST_F(RtlsDriverFixture, writtenZeroBytes)
{
  EXPECT_CALL(*serialInterface_, write(_, _))
      .WillRepeatedly(Return(0));

  EXPECT_FALSE(rtls_->setLabel(0xFEED));
  EXPECT_FALSE(rtls_->setNetworkId(0xFACE));
  EXPECT_FALSE(rtls_->setUpdateTime(2));
  EXPECT_FALSE(rtls_->enableLED());
  EXPECT_FALSE(rtls_->disableLED());

  EXPECT_FALSE(rtls_->setDevice(RtlsDevice::device_type::anchor, 1));
  EXPECT_FALSE(rtls_->setAnchorPosition(1000, 200, 2000));
  EXPECT_FALSE(rtls_->setAnchorHeightForAutoPositioning(1000));
  EXPECT_FALSE(rtls_->enableAnchorInitiator());
  EXPECT_FALSE(rtls_->disableAnchorInitiator());
  EXPECT_FALSE(rtls_->enableAutoAnchorPositioning());
  EXPECT_FALSE(rtls_->disableAutoAnchorPositioning());

  EXPECT_FALSE(rtls_->setDevice(RtlsDevice::device_type::tracker, 1));
  EXPECT_FALSE(rtls_->setTrackerMessageShort());
  EXPECT_FALSE(rtls_->setTrackerMessageLong());
  EXPECT_FALSE(rtls_->enableTrackerStream());

  EXPECT_FALSE(rtls_->restartDevice());
}

TEST_F(RtlsDriverFixture, receivedNack)
{
  EXPECT_CALL(*serialInterface_, read(_, _))
      .WillRepeatedly(Invoke(
         [this](uint8_t* data, size_t size) {
           std::memcpy(data, nackDataBin_, 5);
           return size;
         }));

  EXPECT_FALSE(rtls_->setLabel(0xFEED));
  EXPECT_FALSE(rtls_->setNetworkId(0xFACE));
  EXPECT_FALSE(rtls_->setUpdateTime(4));
  EXPECT_FALSE(rtls_->enableLED());
  EXPECT_FALSE(rtls_->disableLED());

  EXPECT_FALSE(rtls_->setDevice(RtlsDevice::device_type::anchor, 1));
  EXPECT_FALSE(rtls_->setAnchorPosition(1000, 200, 2000));
  EXPECT_FALSE(rtls_->setAnchorHeightForAutoPositioning(1000));
  EXPECT_FALSE(rtls_->enableAnchorInitiator());
  EXPECT_FALSE(rtls_->disableAnchorInitiator());
  EXPECT_FALSE(rtls_->enableAutoAnchorPositioning());
  EXPECT_FALSE(rtls_->disableAutoAnchorPositioning());

  EXPECT_FALSE(rtls_->setDevice(RtlsDevice::device_type::tracker, 1));
  EXPECT_FALSE(rtls_->setTrackerMessageShort());
  EXPECT_FALSE(rtls_->setTrackerMessageLong());
  EXPECT_FALSE(rtls_->enableTrackerStream());

  EXPECT_FALSE(rtls_->restartDevice());
}

TEST_F(RtlsDriverFixture, validUpdateTime)
{
  EXPECT_TRUE(rtls_->setUpdateTime(1));
  EXPECT_EQ(rtls_->getConfig().update_time, 1);
  EXPECT_TRUE(rtls_->setUpdateTime(600));
  EXPECT_EQ(rtls_->getConfig().update_time, 600);
}

TEST_F(RtlsDriverFixture, invalidUpdateTime)
{
  EXPECT_FALSE(rtls_->setUpdateTime(0));
  EXPECT_NE(rtls_->getConfig().update_time, 0);
  EXPECT_FALSE(rtls_->setUpdateTime(601));
  EXPECT_NE(rtls_->getConfig().update_time, 601);
}

TEST_F(RtlsDriverFixture, trackerFailSettingsForAnchor)
{
  EXPECT_TRUE(rtls_->setDevice(RtlsDevice::device_type::tracker, 1));
  EXPECT_FALSE(rtls_->setAnchorPosition(1000, 200, 2000));
  EXPECT_FALSE(rtls_->setAnchorHeightForAutoPositioning(1000));
  EXPECT_FALSE(rtls_->enableAnchorInitiator());
  EXPECT_FALSE(rtls_->disableAnchorInitiator());
  EXPECT_FALSE(rtls_->enableAutoAnchorPositioning());
  EXPECT_FALSE(rtls_->disableAutoAnchorPositioning());
}

TEST_F(RtlsDriverFixture, anchorFailSettingsForTracker)
{
  EXPECT_TRUE(rtls_->setDevice(RtlsDevice::device_type::anchor, 1));
  EXPECT_FALSE(rtls_->setTrackerMessageShort());
  EXPECT_FALSE(rtls_->setTrackerMessageLong());
  EXPECT_TRUE(rtls_->enableTrackerStream()); // possible both in anchor and tracker

  std::vector<uint8_t> expected_data_sent = {0x11, 0x00, 0x42};
  rtls_->disableTrackerStream();
  EXPECT_EQ(data_sent_to_write, expected_data_sent);
}

TEST_F(RtlsDriverFixture, validateMsgBuilding)
{
  std::vector<uint8_t> set_device_sample = {
    static_cast<uint8_t>(RtlsDevice::device_type::tracker), 0x12, 0x34, 0x9A
  };

  std::vector<uint8_t> set_label_sample = {
    static_cast<uint8_t>(RtlsDevice::cmd_header::device_label), 0xC0, 0xFE, 0xCF
  };

  std::vector<uint8_t> set_network_id_sample = {
    static_cast<uint8_t>(RtlsDevice::cmd_header::network_id), 0xDE, 0xAD, 0x9B
  };

  std::vector<uint8_t> set_update_time_sample = {
    static_cast<uint8_t>(RtlsDevice::cmd_header::update_time), 0x00, 0x03, 0xA2
  };

  rtls_->setDevice(RtlsDevice::device_type::tracker, 0x1234);
  EXPECT_EQ(data_sent_to_write, set_device_sample);
  rtls_->setLabel(0xC0FE);
  EXPECT_EQ(data_sent_to_write, set_label_sample);
  rtls_->setNetworkId(0xDEAD);
  EXPECT_EQ(data_sent_to_write, set_network_id_sample);
  rtls_->setUpdateTime(3);
  EXPECT_EQ(data_sent_to_write, set_update_time_sample);
}

TEST_F(RtlsDriverFixture, setAnchorHeight)
{
  std::vector<uint8_t> expected_frame = {
    0x08,
    0x10, 0xF2, 0x77, 0xD5, // z value
    0x28
  };

  int32_t z = 284325845;

  rtls_->setDevice(RtlsDevice::device_type::anchor, 1);
  rtls_->setAnchorHeightForAutoPositioning(z);
  EXPECT_EQ(data_sent_to_write, expected_frame);
  EXPECT_EQ(rtls_->getConfig().anchor_height, z);
}

TEST_F(RtlsDriverFixture, setAnchorPosition)
{
  std::vector<uint8_t> expected_frame = {
    0x06,
    0xD0, 0xF1, 0xDB, 0x05, // x value
    0x32, 0xD3, 0xCF, 0x4B, // y value
    0x00, 0x06, 0xF8, 0x55, // z value
    0x8A
  };

  int32_t x = -789456123;
  int32_t y = 852741963;
  int32_t z = 456789;

  rtls_->setDevice(RtlsDevice::device_type::anchor, 1);
  rtls_->setAnchorPosition(x, y, z);
  EXPECT_EQ(data_sent_to_write, expected_frame);
  EXPECT_EQ(rtls_->getConfig().x, x);
  EXPECT_EQ(rtls_->getConfig().y, y);
  EXPECT_EQ(rtls_->getConfig().z, z);
}

TEST_F(RtlsDriverFixture, readConfigOkSet1)
{
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:1.0",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Anchor",
    "Initiator Mode:Disabled",
    "Device Priority:15987",
    "Device Label:0x1ABE",
    "Network ID:0xC0FE",
    "Network Update Time[ms]:100",
    "Auto Anchor Positioning (AAP):Disabled",
    "Anchor Height for AAP [mm]:1234",
    "Tracker Stream Mode:Disabled",
    "Tracker Message Mode:Long",
    "LED Mode:Disabled",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:204501296",
    "CONFIG:END"
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_TRUE(rtls_->requestConfig());
  EXPECT_EQ(rtls_->getConfig().fw_version, "1.0");
  EXPECT_EQ(rtls_->getConfig().serial_id, "12345678");
  EXPECT_EQ(rtls_->getConfig().type, RtlsDevice::device_type::anchor);
  EXPECT_EQ(rtls_->getConfig().is_initiator, false);
  EXPECT_EQ(rtls_->getConfig().priority, 15987);
  EXPECT_EQ(rtls_->getConfig().label, 0x1ABE);
  EXPECT_EQ(rtls_->getConfig().network_id, 0xC0FE);
  EXPECT_EQ(rtls_->getConfig().update_time, 100);
  EXPECT_EQ(rtls_->getConfig().is_auto_anchor_positioning, false);
  EXPECT_EQ(rtls_->getConfig().anchor_height, 1234);
  EXPECT_EQ(rtls_->getConfig().is_tracker_stream_mode, false);
  EXPECT_EQ(rtls_->getConfig().tracker_message_mode, RtlsDevice::tracker_msg_mode::msg_long);
  EXPECT_EQ(rtls_->getConfig().is_led_enabled, false);
  EXPECT_EQ(rtls_->getConfig().x, 5584200);
  EXPECT_EQ(rtls_->getConfig().y, 1400230);
  EXPECT_EQ(rtls_->getConfig().z, 204501296);
}

TEST_F(RtlsDriverFixture, readConfigOkSet2)
{
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:1.4.9",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Tracker",
    "Initiator Mode:Enabled",
    "Device Priority:65535",
    "Device Label:0x1ABE",
    "Network ID:0xC0FE",
    "Network Update Time[ms]:900",
    "Auto Anchor Positioning (AAP):Enabled",
    "Anchor Height for AAP [mm]:1234",
    "Tracker Stream Mode:Enabled",
    "Tracker Message Mode:Short",
    "LED Mode:Enabled",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:-204501296",
    "CONFIG:END"
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_TRUE(rtls_->requestConfig());
  EXPECT_EQ(rtls_->getConfig().fw_version, "1.4.9");
  EXPECT_EQ(rtls_->getConfig().serial_id, "12345678");
  EXPECT_EQ(rtls_->getConfig().type, RtlsDevice::device_type::tracker);
  EXPECT_EQ(rtls_->getConfig().is_initiator, true);
  EXPECT_EQ(rtls_->getConfig().priority, 65535);
  EXPECT_EQ(rtls_->getConfig().label, 0x1ABE);
  EXPECT_EQ(rtls_->getConfig().network_id, 0xC0FE);
  EXPECT_EQ(rtls_->getConfig().update_time, 900);
  EXPECT_EQ(rtls_->getConfig().is_auto_anchor_positioning, true);
  EXPECT_EQ(rtls_->getConfig().anchor_height, 1234);
  EXPECT_EQ(rtls_->getConfig().is_tracker_stream_mode, true);
  EXPECT_EQ(rtls_->getConfig().tracker_message_mode, RtlsDevice::tracker_msg_mode::msg_short);
  EXPECT_EQ(rtls_->getConfig().is_led_enabled, true);
  EXPECT_EQ(rtls_->getConfig().x, 5584200);
  EXPECT_EQ(rtls_->getConfig().y, 1400230);
  EXPECT_EQ(rtls_->getConfig().z, -204501296);
}

TEST_F(RtlsDriverFixture, readConfigMissingLine)
{
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:1.7.9",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Tracker",
    "Initiator Mode:Enabled",
    "Device Priority:0xFACE",
    "Device Label:0x1ABE",
    "Network Update Time[ms]:900",
    "Auto Anchor Positioning (AAP):Enabled",
    "Tracker Stream Mode:Enabled",
    "Tracker Message Mode:Short",
    "LED Mode:Enabled",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:-204501296",
    "CONFIG:END"
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_FALSE(rtls_->requestConfig());
}

TEST_F(RtlsDriverFixture, readConfigMissingLastLine)
{
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:1.9.9",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Tracker",
    "Initiator Mode:Enabled",
    "Device Priority:0xFACE",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:-204501296",
    ""
    // The implementation of `ISerial::readline` returns an empty string
    // if it didn't read any data - e.g. we are expecting the readConfig to finish
    // either on line containing UART or when there is no mora data in the buffer
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_FALSE(rtls_->requestConfig());
}

TEST_F(RtlsDriverFixture, readConfigIncorrectKeys)
{
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:2.0.0",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Tracker",
    "Initiator Mode:Enabled",
    "Device Priority:0xFACE",
    "Device Label:0x1ABE",
    "Network ID:0xC0FE",
    "Network Update Time[ms]:900",
    "Auto Anchor Positioning (AAP):Enabled",
    "Anchor Height for AAP [mm]:1234",
    "INCORRECT Stream Mode:Enabled",
    "Tracker Message Mode:Short",
    "LED Mode:Enabled",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:-204501296",
    "CONFIG:END"
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_FALSE(rtls_->requestConfig());
}

TEST_F(RtlsDriverFixture, DISABLED_readConfigIncorrectValues)
{
  // TODO: Define input with non-expected value for each parameter
  std::vector<std::string> input = {
    "TERABEE UWB 3D RTLS Firmware Version:2.0.0",
    "Serial ID:12345678",
    "Function Passed.",
    "Device Type:Tracker",
    "Initiator Mode:Enabled",
    "Device Priority:0xFACE",
    "Device Label:0x1ABE",
    "Network ID:0xC0FE",
    "Network Update Time[ms]:900",
    "Auto Anchor Positioning (AAP):Enabled",
    "Anchor Height for AAP [mm]:1234",
    "Tracker Stream Mode:Enabled",
    "Tracker Message Mode:Short",
    "LED Mode:Enabled",
    "Device Position X [mm]:5584200",
    "Device Position Y [mm]:1400230",
    "Device Position Z [mm]:-204501296",
    "CONFIG:END"
  };

  EXPECT_CALL(*serialInterface_, readline(_, _))
    .WillRepeatedly(Invoke(
       [&input](size_t, char) {
         std::string line = input.front();
         input.erase(input.begin());
         return line;
       }));

  EXPECT_FALSE(rtls_->requestConfig());
  EXPECT_EQ(rtls_->getConfig().fw_version, "1.0");
  EXPECT_EQ(rtls_->getConfig().type, RtlsDevice::device_type::anchor);
  EXPECT_EQ(rtls_->getConfig().is_initiator, false);
  EXPECT_EQ(rtls_->getConfig().priority, 1);
  EXPECT_EQ(rtls_->getConfig().label, 0x1ABE);
  EXPECT_EQ(rtls_->getConfig().network_id, 0xC0FE);
  EXPECT_EQ(rtls_->getConfig().update_time, 100);
  EXPECT_EQ(rtls_->getConfig().is_auto_anchor_positioning, false);
  EXPECT_EQ(rtls_->getConfig().anchor_height, 1234);
  EXPECT_EQ(rtls_->getConfig().tracker_message_mode, RtlsDevice::tracker_msg_mode::msg_long);
  EXPECT_EQ(rtls_->getConfig().is_led_enabled, false);
  EXPECT_EQ(rtls_->getConfig().x, 5584200);
  EXPECT_EQ(rtls_->getConfig().y, 1400230);
  EXPECT_EQ(rtls_->getConfig().z, 204501296);
}

TEST_F(RtlsDriverFixture, splitStringValid)
{
  std::string test_string_long_1 = "118334716,D0,0x14B3,0,1800,1300,1433,D1,0xC326,0,0,1600,1897,"
                                   "D2,0xCD0E,1300,0,1600,2660,-855,1149,341";
  std::string test_string_long_2 = "118836472,D0,0xC326,0,0,1600,1911,D1,0xCD0E,1300,0,1600,3063,"
                                   "D2,0x14B3,0,1800,1300,1428,N/A,N/A,N/A";
  std::string test_string_short_1 = "1529569522,-472,1149,293";
  std::string test_string_short_2 = "1529971347,N/A,N/A,N/A";

  std::vector<std::string> fields_long_1 = RtlsDevice::splitString(test_string_long_1, ',');
  std::vector<std::string> fields_long_2 = RtlsDevice::splitString(test_string_long_2, ',');
  std::vector<std::string> fields_short_1 = RtlsDevice::splitString(test_string_short_1, ',');
  std::vector<std::string> fields_short_2 = RtlsDevice::splitString(test_string_short_2, ',');

  EXPECT_EQ(fields_long_1.size(), 22);
  EXPECT_EQ(fields_long_2.size(), 22);
  EXPECT_EQ(fields_short_1.size(), 4);
  EXPECT_EQ(fields_short_2.size(), 4);
}

TEST_F(RtlsDriverFixture, trackerStreamLongLocationValid)
{
  tracker_output_data_ = "118334716,D0,0x14B3,0,1800,1300,1433,D1,0xC326,0,0,1600,1897,"
                         "D2,0xCD0E,1300,0,1600,2660,-855,1149,341";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 118334716);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 3);

  EXPECT_EQ(tracker_msg_.anchors_data.at(0).number, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).id, 0x14B3);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_y, 1800);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_z, 1300);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).distance, 1433);

  EXPECT_EQ(tracker_msg_.anchors_data.at(1).number, 1);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).id, 0xC326);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).distance, 1897);

  EXPECT_EQ(tracker_msg_.anchors_data.at(2).number, 2);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).id, 0xCD0E);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_x, 1300);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).distance, 2660);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), -855);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 1149);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 341);

  EXPECT_EQ(tracker_msg_.is_valid_position, true);
}

TEST_F(RtlsDriverFixture, trackerStreamLongLocationNotComputed)
{
  tracker_output_data_ =
      "118836472,D0,0xC326,0,0,1600,1911,D1,0xCD0E,1300,0,1600,3063,"
      "D2,0x14B3,0,1800,1300,1428,N/A,N/A,N/A";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 118836472);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 3);

  EXPECT_EQ(tracker_msg_.anchors_data.at(0).number, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).id, 0xC326);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).distance, 1911);

  EXPECT_EQ(tracker_msg_.anchors_data.at(1).number, 1);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).id, 0xCD0E);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_x, 1300);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).distance, 3063);

  EXPECT_EQ(tracker_msg_.anchors_data.at(2).number, 2);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).id, 0x14B3);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_y, 1800);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).pos_z, 1300);
  EXPECT_EQ(tracker_msg_.anchors_data.at(2).distance, 1428);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 0);

  EXPECT_EQ(tracker_msg_.is_valid_position, false);
}

TEST_F(RtlsDriverFixture, trackerStreamLongLocationOneAnchor)
{
  tracker_output_data_ =
      "118836472,D0,0xC326,0,0,1600,1911,N/A,N/A,N/A";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 118836472);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 1);

  EXPECT_EQ(tracker_msg_.anchors_data.at(0).number, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).id, 0xC326);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).distance, 1911);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 0);

  EXPECT_EQ(tracker_msg_.is_valid_position, false);
}

TEST_F(RtlsDriverFixture, trackerStreamLongLocationTwoAnchors)
{
  tracker_output_data_ =
      "118836472,D0,0xC326,0,0,1600,1911,D1,0xCD0E,1300,0,1600,3063,N/A,N/A,N/A";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 118836472);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 2);

  EXPECT_EQ(tracker_msg_.anchors_data.at(0).number, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).id, 0xC326);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_x, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(0).distance, 1911);

  EXPECT_EQ(tracker_msg_.anchors_data.at(1).number, 1);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).id, 0xCD0E);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_x, 1300);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_y, 0);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).pos_z, 1600);
  EXPECT_EQ(tracker_msg_.anchors_data.at(1).distance, 3063);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 0);

  EXPECT_EQ(tracker_msg_.is_valid_position, false);
}

TEST_F(RtlsDriverFixture, trackerStreamShortLocationValid)
{
  tracker_output_data_ = "1529569522,-472,1149,293";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 1529569522);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 0);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), -472);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 1149);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 293);

  EXPECT_EQ(tracker_msg_.is_valid_position, true);
}

TEST_F(RtlsDriverFixture, trackerStreamShortLocationNotComputed)
{
  tracker_output_data_ = "1529971347,N/A,N/A,N/A";

  rtls_->startReadingStream();
  waitForTrackerData();
  rtls_->stopReadingStream();

  EXPECT_EQ(tracker_msg_.timestamp, 1529971347);

  EXPECT_EQ(tracker_msg_.anchors_data.size(), 0);

  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(0), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(1), 0);
  EXPECT_EQ(tracker_msg_.tracker_position_xyz.at(2), 0);

  EXPECT_EQ(tracker_msg_.is_valid_position, false);
}

} // namespace terabee

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();

  return ret;
}
