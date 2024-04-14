# Flexo Project

What is "Flexo"?

> Flexo (serial #3370318) is a minor character on Futurama. A Bending Unit, 
Flexo is designed to bend objects such as girders, making him similar to 
Bender but not as evil.

My Flexo is project is a ROS2 project for a humanoid robot that I created
by hacking a Raspberry pi zero 2 W onto a 
[Robosen K1](https://www.robosen.com/us/k1-pro-interstellar-scout). 
A K1 is a 17 servo robot that you can buy for between $199 and $299 depending 
on special deal days. This price is substantially lower than any other servo 
robot kit on the market currently.

However the K1 is designed to be a toy out of the box. There is some
programming support however it is rudamentary and only allows canned routines
to be stored and executed via the mobile phone app. I wanted to create a ROS2
platform that I could develop on and experiment with autonomous like features
and controls, such as dynamic balancing and perhaps some ML based controls.

Ultimately if I ever wish to build multiple robots for something such as
a Robot Cup soccer team I wanted the base cost to be low. This is one reason
to choose the K1, and the fact that I love to take "cheap" hardware and
make it better.

## ROS2 Software

### Build
The resource on a Raspberry Pi Zero 2 W are very minimal and our modern build tools seem to be very heavy by default so you need to run one job on one package at a time. Use the following commands.

```
export MAKEFLAGS="-j1"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --parallel-workers 1 
```

Also there is a problem with the default fastdds loading plugins which are used for managing controller by MoveIt. Changing to cyclonedds seems to fix the problem. Make sure you install the appropriate packages and rebuild the repository.

Install the following
```
apt install ros-rolling-cyclonedds ros-rolling-rmw-cyclonedds-cpp
```

and in your environment
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Hardware Details

### OEM Hardware
The controller that comes in the K1 appears to be based on an STM32F730 in a 100 pin LQFP.

The board contains some sort of IMU, a FET to switch power to the servos, a transceiver to communicate with the servos, and a bluetooth module.

The major components have their informatino lasered off, for what reason we ask?

The servos have a 20 pin micro, that I believe is some cortex M0 vareity such as an STM32F030F4, however the operating voltage and IO is 5V so that doesn't totally make sense, however all the pins on the PCB totally agree with the device pinout.
There is a programming port and I can connect over SWD however the device seems to be locked, so I could not download the firmware. Also one thing that doesnt'
totally make sense is the serial communication pin choice, it is not connected to a UART capable pin.

## Communications

The communication with the servos is half-duplex 5V at 115200 baud.

I have implemented enable/disable, position read back, and broadcast position updates, also their CRC.

Further research is necessary to determine if it is possible to program each servo with an address. Being able to change a servo's address would faciliate easy replacements by parting out another K1 for replacement servos.

## Flex Hat for Raspberry Pi

I have designed and manufactured a custom hat for a Raspberry Pi Zero 2 W. The board takes the internal 7.2V battery and regulates that down to 5V to power the Pi board.
It also has an integrated power controller for the servo power rail, with soft start and over current protection.
Additionally there is a half duplex transceiver that also translates the voltage from 3.3V to 5V for the servo comms. The half duplex is controlled by a pin coming from the Pi that is controlled by the kernel automatically.

The schematic will be released at some point, once things work.

