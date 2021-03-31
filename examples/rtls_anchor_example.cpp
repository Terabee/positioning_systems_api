#include <memory>
#include <iostream>
#include "rtls_driver/rtls_driver.hpp"

#ifdef __linux__
  #include "serial_communication/serial.hpp"
  using SerialInterface = terabee::serial_communication::Serial;
#elif defined(__MINGW32__) || defined(__MINGW64__) || defined(_WIN32) || defined(_WIN64)
  #include "serial_communication/serial_windows.hpp"
  using SerialInterface = terabee::serial_communication::SerialWindows;
#endif

int main(int argc, char **argv)
{
  terabee::utility::logger::Logger logger("AnchorExample");
  if (argc != 2)
  {
    logger->info("usage: ./rtls_anchor_example PORT_NAME");
    return -1;
  }

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

  rtls_device.setDevice(terabee::RtlsDevice::device_type::anchor, 0);
  rtls_device.setAnchorPosition(0, 0, 0);
  rtls_device.setLabel(0x1ABE);
  rtls_device.setUpdateTime(1);
  rtls_device.setNetworkId(0xC0FE);
  rtls_device.setAnchorHeightForAutoPositioning(1320);
  rtls_device.enableAnchorInitiator();
  rtls_device.disableAutoAnchorPositioning();
  rtls_device.enableLED();
  rtls_device.requestConfig();
  terabee::RtlsDevice::config_t device_configuration = rtls_device.getConfig();

  return serial_port->close() ? 0 : -1;
}
