# positioning_systems_api

This package comprises following modules described below: **serial_communication**, **follow_me_driver**, **rtls_driver**, **logger**.

## serial_communication

General purpose serial communication

### Features

 - support for different baudrate, parity, bytesize (5, 6, 7, or 8 bits), stopbits and flow control
 - implementation for Linux and Windows

### Usage

In order to use serial_communication in your code, include the interface header and appropriate implementation header, e.g.

```
#include "serial_communication/iserial.hpp"
#include "serial_communication/serial.hpp"
```

Add `positioning_systems_api` to `find_package` of your `CMakeLists.txt` and link the target against `serial_communication`:

```
target_link_libraries(target_name
  positioning_systems_api::serial_communication
)
```

See subdirectory `examples` for a C++ source file presenting usage of `serial_communication`.

## follow_me_driver

Driver dedicated for communication with Terabee Follow-Me system

### Features

 - set configuration parameters: span manual setting and auto-calibration, output mode (text/binary), swap beacons, Exponential Moving Average filter window size,
 - configuration of RS485 connection parameters (slave address, baud rate, parity)
 - set configuration parameters of the remote control: enable/disable buzzer, change button mode (toggle/hold)
 - data reception (distance and heading)

### Usage

Include the following header: 
```
#include "follow_me_driver/follow_me_driver.hpp"
```

and header with appropriate implementation of serial port, e.g.
```
#include "serial_communication/serial.hpp"
```

Add `positioning_systems_api` to `find_package` of your `CMakeLists.txt` and link the target against `follow_me_driver`:

```
target_link_libraries(target_name
  positioning_systems_api::follow_me_driver
)
```

See subdirectory `examples` for a C++ source file presenting usage of `follow_me_driver`.

## rtls_driver

Driver dedicated for communication with Terabee Robot Positioning System

### Features

- setting all configuration parameters
- reading the whole configuration of device
- reading position output of the tracker

### Usage

Include the following header: 
```
#include "rtls_driver/rtls_driver.hpp"
```

and header with appropriate implementation of serial port, e.g.
```
#include "serial_communication/serial.hpp"
```

Add `positioning_systems_api` to `find_package` of your `CMakeLists.txt` and link the target against `rtls_driver`:

```
target_link_libraries(target_name
  positioning_systems_api::rtls_driver
)
```

See subdirectory `examples` for a C++ source file presenting usage of `rtls_driver`.

## Compilation

```
mkdir build
cd build
cmake ../
make
```

**Note:**

If using MinGW in Windows, type

`cmake .. -G "MinGW Makefiles"` instead of `cmake ../`

to prevent configuring for MSVC.

Use `cmake.exe --build . --target all -- -j` to build.

## Installation in the system
```
sudo make install
sudo ldconfig /usr/local/lib
```

## Run tests

```
cd build
make test
```
For tests to pass, linux null modem emulator is required https://github.com/freemed/tty0tty

## Logging
To use the logger, set the following environment variables:
```
export LOGGER_ENABLE_LOGGING=1
export LOGGER_PRINT_STDOUT=1
```

Set `export LOGGER_ENABLE_DEBUG=1` to enable debug logs.

Set `export LOGGER_FILENAME=rtls_output.log` to change the default log file name.
