/*
 * Arduino micro servo controller node
 */


#include <Arduino.h>
#include "arduino_micro_ros.h"
#include "ros_servo_controller.hpp"


ros::NodeHandle node_handle;
ros_servo_controller::RosServoController<9, 10, 11, 12> servo_controller(&node_handle);


void setup()
{
  node_handle.initNode();
  servo_controller.setup();
}


void loop()
{
  node_handle.spinOnce();
  servo_controller.spinOnce();
}
