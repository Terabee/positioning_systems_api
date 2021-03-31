#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include "follow_me_driver/follow_me_driver.hpp"

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
  terabee::utility::logger::Logger logger("FollowMeRemoteExample");
  if (argc != 2)
  {
    logger->info("usage: ./follow_me_remote_example PORT_NAME");
    return -1;
  }
  std::signal(STOP_SIGNAL, signal_handler);

  std::string portname = argv[1];
  std::shared_ptr<terabee::serial_communication::ISerial> serial_port =
    std::make_shared<SerialInterface>(portname);

  serial_port->setBaudrate(115200);
  serial_port->setTimeout(std::chrono::milliseconds(700));

  serial_port->open();

  if (!serial_port->isOpen())
  {
    logger->error("Failed to open serial port!");
    return 0;
  }

  terabee::FollowMeRemoteControl remote_control(serial_port);

  remote_control.setBuzzer(false);
  remote_control.setButtonMode(terabee::FollowMeRemoteControl::button_mode::hold);

  terabee::FollowMeRemoteControl::button_mode button_mode;
  bool buzzer;
  remote_control.retrieveRemoteParameters(button_mode, buzzer);
  logger->info("Button mode: {}", static_cast<int>(button_mode));
  logger->info("Buzzer: {}", buzzer);

  return serial_port->close() ? 0 : -1;
}
