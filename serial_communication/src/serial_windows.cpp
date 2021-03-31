#include "serial_communication/serial_windows.hpp"

#include <windows.h>

#include <cerrno>
#include <thread>
#include <cstring>
#include <iostream>

namespace terabee
{
namespace serial_communication
{

SerialWindows::SerialWindows(const std::string &port):
  logger_("Serial(" + port + ")"),
  is_open_(false),
  portname_(port),
  baudrate_(CBR_9600),
  parity_(NOPARITY),
  stop_bits_(ONESTOPBIT),
  byte_size_(DATABITS_8),
  timeout_(0),
  blocking_(false),
  serial_handle_()
{
  setPortName(portname_);
  setBaudrate(9600);
  setParity(parity_t::parity_none);
  setBytesize(bytesize_t::eightbits);
  setStopbits(stopbits_t::stopbits_one);
  setFlowcontrol(flowcontrol_t::flowcontrol_none);
  logger_->debug("Serial::Serial(" + portname_ + ")");
}

SerialWindows::~SerialWindows()
{
  logger_->debug("SerialWindows::~SerialWindows()");
  close();
}

bool SerialWindows::open()
{
  if (isOpen())
  {
    logger_->error("Cannot open port. Already open!");
    return false;
  }

  std::string port_path = "\\\\.\\" + portname_;
  logger_->info("Port Name: {}", port_path);
  serial_handle_ = CreateFile(port_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  if (serial_handle_ == INVALID_HANDLE_VALUE)
  {
    logger_->error("Port cannot be opened: INVALID_HANDLE_VALUE!");
    return false;
  }

  DCB serialParams = { 0 };
  serialParams.DCBlength = sizeof(serialParams);
  if (GetCommState(serial_handle_, &serialParams) == FALSE)
  {
    logger_->error("Error in GetCommmState!");
    return false;
  }

  serialParams.BaudRate = baudrate_;
  serialParams.ByteSize = byte_size_;
  serialParams.StopBits = stop_bits_;
  serialParams.Parity = parity_;
  if (SetCommState(serial_handle_, &serialParams) == FALSE)
  {
    logger_->error("Error in SetCommState!");
    return false;
  }

  COMMTIMEOUTS timeout = { 0 };
  timeout.ReadIntervalTimeout = 50;
  timeout.ReadTotalTimeoutConstant = timeout_.count();
  timeout.ReadTotalTimeoutMultiplier = 50;
  timeout.WriteTotalTimeoutConstant = 50;
  timeout.WriteTotalTimeoutMultiplier = 10;
  if (SetCommTimeouts(serial_handle_, &timeout) == FALSE)
  {
    logger_->error("Error in SetCommTimeouts!");
    return false;
  }
  is_open_ = true;
  return is_open_;
}

bool SerialWindows::close()
{
  if (!isOpen())
  {
    logger_->warn("Cannot close port, because is not open!");
    return false;
  }

  if (CloseHandle(serial_handle_) == FALSE)
  {
    logger_->error("Error in CloseHandle!");
    return false;
  }
  is_open_ = false;
  return true;
}

bool SerialWindows::clear()
{
  return PurgeComm(serial_handle_, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
}

bool SerialWindows::isOpen()
{
  return is_open_;
}

size_t SerialWindows::available()
{
  logger_->error("available() Not supported!");
  return 0;
}

bool SerialWindows::setPortName(const std::string &portname)
{
  if (isOpen())
  {
    logger_->error("Cannot change port name on the open port!");
    return false;
  }
  portname_ = portname;
  return true;
}

bool SerialWindows::setBaudrate(uint32_t baudrate)
{
  if(isOpen())
  {
    logger_->error("Cannot change settings on the open port!");
    return false;
  }

  switch (baudrate)
  {
  case 1200:
    baudrate_ = CBR_1200;
    break;
  case 2400:
    baudrate_ = CBR_2400;
    break;
  case 4800:
    baudrate_ = CBR_4800;
    break;
  case 9600:
    baudrate_ = CBR_9600;
    break;
  case 19200:
    baudrate_ = CBR_19200;
    break;
  case 38400:
    baudrate_ = CBR_38400;
    break;
  case 57600:
    baudrate_ = CBR_57600;
    break;
  case 115200:
    baudrate_ = CBR_115200;
    break;
  default:
    logger_->error("Unsupported baudrate: {}!", baudrate);
    return false;
  }

  return true;
}

bool SerialWindows::setParity(parity_t iparity)
{
  if(isOpen())
  {
    logger_->error("Cannot change settings on the open port!");
    return false;
  }

  switch (iparity)
  {
  case parity_t::parity_none:
    parity_ = NOPARITY;
    break;
  case parity_t::parity_odd:
    parity_ = ODDPARITY;
    break;
  case parity_t::parity_even:
    parity_ = EVENPARITY;
    break;
  case parity_t::parity_mark:
    parity_ = MARKPARITY;
    break;
  case parity_t::parity_space:
    parity_ = SPACEPARITY;
    break;
  default:
    logger_->error("Unsupported parity: {}!", static_cast<int>(iparity));
    return false;
  }

  return true;
}

bool SerialWindows::setBytesize(bytesize_t ibytesize)
{
  if(isOpen())
  {
    logger_->error("Cannot change settings on the open port!");
    return false;
  }

  switch (ibytesize)
  {
  case bytesize_t::fivebits:
    byte_size_ = DATABITS_5;
    break;
  case bytesize_t::sixbits:
    byte_size_ = DATABITS_6;
    break;
  case bytesize_t::sevenbits:
    byte_size_ = DATABITS_7;
    break;
  case bytesize_t::eightbits:
    byte_size_ = DATABITS_8;
    break;
  default:
    logger_->error("Unsupported bytesize: {}!", static_cast<int>(ibytesize));
    return false;
  }

  return true;
}

bool SerialWindows::setStopbits(stopbits_t istopbits)
{
  if(isOpen())
  {
    logger_->error("Cannot change settings on the open port!");
    return false;
  }

  switch (istopbits)
  {
  case stopbits_t::stopbits_one:
    stop_bits_ = ONESTOPBIT;
    break;
  case stopbits_t::stopbits_two:
    stop_bits_ = TWOSTOPBITS;
    break;
  case stopbits_t::stopbits_one_point_five:
    stop_bits_ = ONE5STOPBITS;
    break;
  default:
    logger_->error("Unsupported stopbits: {}!", static_cast<int>(istopbits));
    return false;
  }

  return true;
}

bool SerialWindows::setFlowcontrol(flowcontrol_t iflowcontrol)
{
  logger_->error("setFlowcontrol: Not Implemented!");
  return true;
}

bool SerialWindows::setTimeout(std::chrono::milliseconds timeout)
{
  if(isOpen())
  {
    logger_->error("Cannot change settings on the open port!");
    return false;
  }
  timeout_ = timeout;
  if (timeout_.count() > 0)
  {
    blocking_ = true;
  }
  return true;
}

std::string SerialWindows::readline(size_t max_buffer_size, char eol = '\n')
{
  if (!isOpen())
  {
    logger_->error("Cannot read because not open!");
    return "";
  }
  std::string result;
  uint8_t character = 0;
  const uint8_t bytes_number = 1;

  for (size_t i = 0; i < max_buffer_size; i++)
  {
    size_t num_read = read(&character, bytes_number);
    if (num_read < bytes_number)
    {
      logger_->error("Failed to read: {}!", std::strerror(errno));
      return "";
    }
    if (character == eol) break;
    result += static_cast<char>(character);
  }
  return result;
}

bool SerialWindows::flushInput()
{
  if (!isOpen())
  {
    logger_->error("Cannot flush because not open!");
    return false;
  }
  return (0 != PurgeComm(serial_handle_, PURGE_RXABORT|PURGE_RXCLEAR));
}

bool SerialWindows::flushOutput()
{
  if (!isOpen())
  {
    logger_->error("Cannot flush because not open!");
    return false;
  }
  return (0 != PurgeComm(serial_handle_, PURGE_TXABORT|PURGE_TXCLEAR));
}

size_t SerialWindows::write(const uint8_t *data, size_t size)
{
  if (!isOpen())
  {
    logger_->error("Cannot write because not open!");
    return 0;
  }
  DWORD numBytesSent;
  if (WriteFile(serial_handle_, data, size, &numBytesSent, NULL) == FALSE)
  {
    logger_->error("Error in WriteFile!");
    return 0;
  }
  if (numBytesSent <= 0)
  {
    logger_->error("Failed to write: {}!", std::strerror(errno));
  }
  return (numBytesSent >= 0) ? numBytesSent : 0;
}

size_t SerialWindows::read(uint8_t *buffer, size_t size)
{
  if (!isOpen())
  {
    logger_->error("Cannot read because not open!");
    return 0;
  }
  size_t numBytesRead = 0;
  if (blocking_)
  {
    auto start = std::chrono::high_resolution_clock::now();
    do
    {
      DWORD nRead;
      bool retval = ReadFile(serial_handle_, buffer + numBytesRead, size - numBytesRead,
                            &nRead, NULL);
      if (retval)
      {
        numBytesRead += static_cast<size_t>(nRead);
      }
      else
      {
        logger_->error("Failed to read. Error code: {}!", GetLastError());
        return numBytesRead;
      }
      int timeToWait = 1e6*(size-numBytesRead)/(baudrate_);
      std::this_thread::sleep_for(std::chrono::microseconds(timeToWait));
      if ((timeout_.count() != 0) && (std::chrono::high_resolution_clock::now() - start > timeout_))
      {
        logger_->error("Read timeout!");
        return numBytesRead;
      }
    } while (numBytesRead < size);
  }
  else
  {
    DWORD nRead;
    numBytesRead = ReadFile(serial_handle_, buffer, size,
                            &nRead, NULL);
  }
  return numBytesRead;
}

}  // namespace serial_communication
}  // namespace terabee
