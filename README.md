# MegaIndQt

MegaIndQt is a Qt/C++ wrapper for industrial automation boards manufactured by Sequent Microsystems. It provides a QObject-based API for communicating with the board, controlling its I/O, and receiving device updates through Qt signals.

The megaind-core code can be found in this repository:
[SequentMicrosystems/megaind-rpi](https://github.com/SequentMicrosystems/megaind-rpi).
The OG coder [Alexandru Burcea](https://github.com/alexburcea2877).
The included upstream code is licensed under the MIT License.

## Features

- Connect to an "Industrial Automation Stackable HAT for Raspberry Pi" and read board information, including firmware version and diagnostic voltages.
- Read and set analog inputs and outputs: 0–10 V, ±10 V, and 4–20 mA.
- Read opto-isolated digital inputs, configure edge detection, and access counters and frequency readings.
- Control digital outputs, LEDs, and open-drain PWM.
- Scan 1-Wire devices and read sensor ROM IDs and temperatures.
- Read and set the RTC.
- Configure and query the watchdog.
- Configure RS-485 modes and Modbus slave settings.
- Calibrate analog inputs and outputs.
- Poll device values at configurable intervals and receive updates through Qt signals.

## Requirements

- CMake 3.20 or newer
- A C99 and C++17 compiler
- Qt 6 Core
- A POSIX-compatible system with `pthread` and the math library
- A compatible MegaInd board for hardware communication

## Build

```sh
cmake -S . -B build
cmake --build build
```

The build produces the static libraries `MegaIndQt` and `megaind_core`.

## Use from another CMake project

Add this repository as a subdirectory and link your application to `MegaIndQt`:

```cmake
add_subdirectory(path/to/megaind-qt)

target_link_libraries(my_app PRIVATE MegaIndQt)
```

The public header is `megaind/megaind_device.hpp`.

## Minimal example

```cpp
#include <QCoreApplication>
#include <QDebug>
#include <megaind/megaind_device.hpp>

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    MegaInd::MegaIndDevice device(MegaInd::MegaIndDevice::Board0);

    if (!device.open())
    {
        qCritical() << "Could not open the MegaInd board";
        return 1;
    }

    const float voltage = device.getVoltageIn(MegaInd::MegaIndDevice::Channel1);
    qInfo() << "Channel 1 voltage:" << voltage << "V";

    device.close();
    return 0;
}
```

Call `open()` before using hardware operations. To receive periodic updates, configure polling intervals if needed and call `startPolling()`. Connect to `MegaIndDevice` signals to handle connection changes, errors, input changes, counters, analog readings, and 1-Wire sensor events.
