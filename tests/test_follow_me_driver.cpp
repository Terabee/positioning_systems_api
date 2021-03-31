#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <future>
#include <cstring>
#include "follow_me_driver/follow_me_driver.hpp"

#include "MockISerial.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;

namespace terabee {

class FollowMeDriverFixture : public testing::Test
{
protected:
  FollowMeDriverFixture():
    serialInterface_(std::make_shared<serial_communication::MockISerial>())
  {
    ON_CALL(*serialInterface_, open()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, isOpen()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, available()).WillByDefault(Return(69));
    ON_CALL(*serialInterface_, readline(_, _)).WillByDefault(Return(fakeDataText));
    ON_CALL(*serialInterface_, write(_, _))
        .WillByDefault(Invoke(
           [this](const uint8_t* data, size_t size) {
             data_sent_to_write.assign(data, data + size);
             return size;
           }));
    ON_CALL(*serialInterface_, read(_, _))
        .WillByDefault(Invoke(
           [this](uint8_t* data, size_t size) {
             std::memcpy(data, dummyAck, 4);
             return size;
           }));

    followMe_.reset(new FollowMeMasterBeacon(serialInterface_));
    remoteControl_.reset(new FollowMeRemoteControl(serialInterface_));
  }
  virtual ~FollowMeDriverFixture();

  std::unique_ptr<FollowMeMasterBeacon> followMe_;
  std::unique_ptr<FollowMeRemoteControl> remoteControl_;

  std::shared_ptr<serial_communication::MockISerial> serialInterface_;
  std::vector<uint8_t> data_sent_to_write;
  uint8_t fakeDataBin_[6] = {'F', 'M', 0x02, 0x04, 0xF1, 0xBD};
  uint8_t dummyAck[4] = { 0x00, 0x03, 0x01, 0x38 };
  std::string fakeDataText = "FM\t2503\t79\n";

  uint8_t ack_printout_text[4] = { 0x00, 0x01, 0x00, 0x15 };

  // TEST Frames
  const uint8_t CMD_PRINTOUT_TEXT[4] = { 0x00, 0x11, 0x01, 0x45 };
  const uint8_t CMD_PRINTOUT_BINARY[4] = { 0x00, 0x11, 0x02, 0x4C };
  const uint8_t CMD_DEACTIVATE_DEBUG[7] = { 0x00, 0x54, 0x0D, 0x00, 0x00, 0x00, 0x8B };
  const uint8_t CMD_ACTIVATE_DEBUG[7] = { 0x00, 0x54, 0x0D, 0x01, 0x00, 0x00, 0xE0 };
  const uint8_t CMD_AUTOCALIBRATE[5] = { 0x00, 0x22, 0x00, 0x00, 0x95 };
  const uint8_t CMD_EMA_WINDOW[5] = { 0x00, 0x52, 0x01, 0x0A, 0xD1 };

public:

};

FollowMeDriverFixture::~FollowMeDriverFixture() {
  followMe_.reset();
}

TEST_F(FollowMeDriverFixture, CalcCmdLen)
{
  EXPECT_EQ(4, followMe_->calculateCommandLength(CMD_PRINTOUT_TEXT));
  EXPECT_EQ(4, followMe_->calculateCommandLength(CMD_PRINTOUT_BINARY));
  EXPECT_EQ(7, followMe_->calculateCommandLength(CMD_DEACTIVATE_DEBUG));
  EXPECT_EQ(7, followMe_->calculateCommandLength(CMD_ACTIVATE_DEBUG));
  EXPECT_EQ(5, followMe_->calculateCommandLength(CMD_AUTOCALIBRATE));
  EXPECT_EQ(5, followMe_->calculateCommandLength(CMD_EMA_WINDOW));
}

TEST_F(FollowMeDriverFixture, ProcessFrameText)
{
  EXPECT_CALL(*serialInterface_, read(_,_))
      .WillRepeatedly(Invoke(
        [this](uint8_t* data, size_t size) {
          std::memcpy(data, ack_printout_text, 4);
          return size;
        }));
  followMe_->printoutModeText();

  PolarPoint2D ref_point = { 2.503f, 1.3788f };
  PolarPoint2D measured_point;

  followMe_->process(measured_point);

  EXPECT_NEAR(ref_point.distance, measured_point.distance, 1e-4);
  EXPECT_NEAR(ref_point.heading, measured_point.heading, 1e-4);
}

TEST_F(FollowMeDriverFixture, ProcessFrameBin)
{
  EXPECT_CALL(*serialInterface_, read(_,_))
      .WillRepeatedly(Invoke(
        [this](uint8_t* data, size_t size) {
          std::memcpy(data, ack_printout_text, 4);
          return size;
        }));
  followMe_->printoutModeBin();

  PolarPoint2D ref_point = { 0.516f, -0.26179f };
  PolarPoint2D measured_point;

  EXPECT_CALL(*serialInterface_, available()).WillRepeatedly(Return(7));
  EXPECT_CALL(*serialInterface_, read(_, _))
      .WillRepeatedly(Invoke(
        [this](uint8_t* data, size_t size) {
          std::memcpy(data, fakeDataBin_, 7);
          return size;
        }));
  followMe_->process(measured_point);

  EXPECT_NEAR(ref_point.distance, measured_point.distance, 1e-4);
  EXPECT_NEAR(ref_point.heading, measured_point.heading, 1e-4);
}

TEST_F(FollowMeDriverFixture, commandSwapBeacons)
{
  std::vector<uint8_t> expected_swap_true = { 0x00, 0x31, 0x01, 0xEB };
  std::vector<uint8_t> expected_swap_false = { 0x00, 0x31, 0x00, 0xEC };

  followMe_->swapBeacons(true);
  EXPECT_EQ(data_sent_to_write, expected_swap_true);
  followMe_->swapBeacons(false);
  EXPECT_EQ(data_sent_to_write, expected_swap_false);
}

TEST_F(FollowMeDriverFixture, commandBeaconsSpan)
{
  std::vector<uint8_t> expected_frame = { 0x00, 0x22, 0x0B, 0xAB, 0x5A };

  followMe_->setBeaconsSpan(2987);
  EXPECT_EQ(data_sent_to_write, expected_frame);
}

TEST_F(FollowMeDriverFixture, commandEMAWindow)
{
  std::vector<uint8_t> expected_frame = { 0x00, 0x52, 0x01, 0x17, 0x82 };

  followMe_->setEMAWindow(23);
  EXPECT_EQ(data_sent_to_write, expected_frame);
}

TEST_F(FollowMeDriverFixture, commandRS485_Parameters)
{
  std::vector<uint8_t> expected_frame_baud_parity =
    { 0x00, 0x55, 0x02, 0x01, 0xC2, 0x00, 0x01, 0xEB };

  followMe_->setRS485_Parameters(115200, FollowMeMasterBeacon::rs485_parity::rs485_parity_odd);
  EXPECT_EQ(data_sent_to_write, expected_frame_baud_parity);
}

TEST_F(FollowMeDriverFixture, commandRS485_SlaveId)
{
  std::vector<uint8_t> expected_frame_slave_id =
    { 0x00, 0x52, 0x03, 0x17, 0xA8 };

  followMe_->setRS485_SlaveId(23);
  EXPECT_EQ(data_sent_to_write, expected_frame_slave_id);
}

TEST_F(FollowMeDriverFixture, remoteButtonMode)
{
  std::vector<uint8_t> expected_toggle = { 0x08, 0x01, 0xAF };
  std::vector<uint8_t> expected_hold = { 0x08, 0x00, 0xA8 };

  remoteControl_->setButtonMode(FollowMeRemoteControl::button_mode::toggle);
  EXPECT_EQ(data_sent_to_write, expected_toggle);

  remoteControl_->setButtonMode(FollowMeRemoteControl::button_mode::hold);
  EXPECT_EQ(data_sent_to_write, expected_hold);
}

TEST_F(FollowMeDriverFixture, remoteBuzzer)
{
  std::vector<uint8_t> expected_enabled = { 0x09, 0x01, 0xBA };
  std::vector<uint8_t> expected_disabled = { 0x09, 0x00, 0xBD };

  remoteControl_->setBuzzer(true);
  EXPECT_EQ(data_sent_to_write, expected_enabled);

  remoteControl_->setBuzzer(false);
  EXPECT_EQ(data_sent_to_write, expected_disabled);
}

} // namespace terabee

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();

  return ret;
}
