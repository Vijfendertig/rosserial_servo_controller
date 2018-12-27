# Rosserial servo controller node (Arduino Micro)

This package builds a rosserial compatible USB servo controller from an Arduino Micro.

The rosserial node provides `/servo/control` and `/servo/control_raw` subscribers which receive target positions (relative positions (-1000 to 10000) or absolute pulse widths (in µs)) for the different servos.

## Dependencies

- [ROS](http://www.ros.org/). I used Melodic Morenia on Ubuntu 18.04 LTS, but other versions might work too.
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).
- [Arduino IDE](https://www.arduino.cc/en/Main/Software). I used version 1.8.8. Other versions might work too, but the one included in Ubuntu 18.04 LTS doesn't.
- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake). I [forked the repository and added a patch](https://github.com/Vijfendertig/arduino-cmake) to use `avr-gcc` and `avr-g++` from the Arduino IDE rather than the one provided with Ubuntu 18.04 LTS.

The (patched) arduino-cmake dependency is included as git submodules.

## Building

Include the package in a ROS workspace. Both building (messages, firmware...) and uploading the firmware is done using catkin_make.

Due to some internal details of rosserial_arduino's make_libraries.py script, building the package isn't as straightforward as I would like it to be. The problem is that to create our custom messages in the Arduino ros_lib library, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't available until the build is finished. See [https://github.com/ros-drivers/rosserial/issues/239] for more details. 
The most elegant workaround I found is to exclude the firmware from the default catkin_make (or CMake) target and build it manually afterwards.

So, to build the package including the firmware for the Arduino Micro, run:

- `catkin_make -DARDUINO_SDK_PATH=/opt/arduino-1.8.8` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the setup script for your favourite shell)
- `catkin_make ros_servo_controller_firmware_arduino_micro` (to build the firmware)
- `catkin_make ros_servo_controller_firmware_arduino_micro-upload` (to upload the firmware to your Arduino Micro)

## Running

...

## License

MIT license, see LICENSE.md for details.

Git submodules:

- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake): Unknown