#include <memory>
#include <thread>
#include <csignal>
#include <iostream>
#include "rtls_driver/rtls_driver.hpp"

#ifdef __linux__
  #define STOP_SIGNAL SIGTSTP
  #include "serial_communication/serial.hpp"
  using SerialInterface = terabee::serial_communication::Serial;
#elif defined(__MINGW32__) || defined(__MINGW64__) || defined(_WIN32) || defined(_WIN64)
  #define STOP_SIGNAL SIGTERM
  #include "serial_communication/serial_windows.hpp"
  using SerialInterface = terabee::serial_communication::SerialWindows;
#endif

static volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char **argv)
{
  terabee::utility::logger::Logger logger("TrackerExample");
  if (argc != 2)
  {
    logger->info("Usage: ./rtls_tracker_example PORT_NAME");
    return -1;
  }

  std::signal(STOP_SIGNAL, signal_handler);
  std::string portname = argv[1];
  std::shared_ptr<terabee::serial_communication::ISerial> serial_port =
    std::make_shared<SerialInterface>(portname);

  serial_port->setBaudrate(115200);
  serial_port->setTimeout(std::chrono::milliseconds(800));

  serial_port->open();

  if (!serial_port->isOpen())
  {
    logger->error("Failed to open serial port!");
    return 0;
  }

  terabee::RtlsDevice rtls_device(serial_port);

  rtls_device.disableTrackerStream();
  serial_port->flushInput();
  rtls_device.setDevice(terabee::RtlsDevice::device_type::tracker, 1);
  rtls_device.setLabel(0x1ABE);
  rtls_device.setUpdateTime(1);
  rtls_device.setNetworkId(0xC0FE);
  rtls_device.setTrackerMessageLong();
  rtls_device.enableLED();
  rtls_device.requestConfig();
  terabee::RtlsDevice::config_t device_configuration = rtls_device.getConfig();

  rtls_device.enableTrackerStream();

  logger->info("Press Ctrl + Z to quit");

  terabee::RtlsDevice::tracker_msg_t tracker_data;
  rtls_device.registerOnDistanceDataCaptureCallback(
    [&tracker_data, &logger](const terabee::RtlsDevice::tracker_msg_t& tracker_msg)
    {
      logger->info("T:{}", tracker_msg.timestamp);
      if (tracker_msg.is_valid_position)
      {
        logger->info("Tracker position: ({}, {}, {})",
          tracker_msg.tracker_position_xyz.at(0),
          tracker_msg.tracker_position_xyz.at(1),
          tracker_msg.tracker_position_xyz.at(2));
      }
      tracker_data = tracker_msg;
    });
  rtls_device.startReadingStream();

  while (g_signal_status != STOP_SIGNAL)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  rtls_device.stopReadingStream();
  return serial_port->close() ? 0 : -1;
}
