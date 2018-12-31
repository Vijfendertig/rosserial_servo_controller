cmake_minimum_required(VERSION 3.1)

# Compile as C++11.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Arduino CMake included with ROS Melodic Morenia assumes the hardware platform path (where the
# boards.txt file lives) is ${ARDUINO_SDK_PATH}/hardware/arduino. At least for my version 1.8.8,
# this should be ${ARDUINO_SDK_PATH}/hardware/arduino/avr.
register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

include_directories(
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries
  ${ARDUINO_SDK_PATH}/libraries
  ${ARDUINO_SDK_PATH}/libraries/Servo/src
  #/opt/arduino-1.8.8/libraries/Servo/src/Servo.h
  ${ROS_LIB_DIR}
)

add_definitions(-DUSB_CON -DARDUINO_ARCH_AVR)

link_directories(
  ${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries
)

generate_arduino_firmware(arduino_micro
  SRCS arduino_micro_servo_controller_node.cpp
       configured_servo.cpp
       ${ARDUINO_SDK_PATH}/libraries/Servo/src/avr/Servo.cpp
       ${ROS_LIB_DIR}/time.cpp
  ARDLIBS Servo
  BOARD micro
  PROGRAMMER arduinoisp
  PORT /dev/ttyACM0
)
