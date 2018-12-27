#ifndef __ROS_SERVO_CONTROLLER__
#define __ROS_SERVO_CONTROLLER__


#include "arduino_micro_ros.h"
#include <Servo.h>
#include <ros_servo_controller/ControlServo.h>
#include <ros_servo_controller/ControlServoRaw.h>
#include <ros_servo_controller/ConfigureServo.h>


namespace ros_servo_controller {

  class ConfiguredServo {
    private:
      uint8_t pin_number_;
      uint16_t pulse_width_minimum_;
      uint16_t pulse_width_center_;
      uint16_t pulse_width_maximum_;
      uint16_t pulse_width_initial_;
      bool pulse_width_enforce_limits_;
      Servo servo_;
    public:
      ConfiguredServo(uint8_t pin_number);
      ~ConfiguredServo();
      void setup();
      void setPosition(int16_t position);
      void setPulseWidth(uint16_t pulse_width);
    private:
      uint16_t correctPulseWidth(uint16_t pulse_width_requested) const;
  };


  template <int ...pin_numbers>
  class RosServoController {
    private:  // Data types and member variables.
      const uint8_t servo_count_;
      int pin_numbers_[sizeof...(pin_numbers)];
      ConfiguredServo servos_[sizeof...(pin_numbers)];
      ros::NodeHandle * node_handle_;
      ros::Subscriber<ControlServo, RosServoController> control_subscriber_;
      ros::Subscriber<ControlServoRaw, RosServoController<pin_numbers...>> control_raw_subscriber_;
      // ros::ServiceServer<ConfigureServoRequest, ConfigureServoResponse, RosServoController> configure_service_;
    public:  // Member functions.
      RosServoController(ros::NodeHandle * node_handle);
      ~RosServoController() = default;
      void setup();
    private:  // Member functions.
      void controlCallback(const ControlServo & message);
      void controlRawCallback(const ControlServoRaw & message);
      void configureCallback(const ConfigureServoRequest & request, ConfigureServoResponse & response);
      void updateServoCommands();
  };


  template<int ...pin_numbers>
  RosServoController<pin_numbers...>::RosServoController(ros::NodeHandle * node_handle):
    servo_count_{sizeof...(pin_numbers)},
    pin_numbers_{pin_numbers...},
    servos_{pin_numbers...},
    node_handle_{node_handle},
    control_subscriber_{"/servos/control", &RosServoController::controlCallback, this},
    control_raw_subscriber_{"/servos/control_raw", &RosServoController<pin_numbers...>::controlRawCallback, this}
    // configure_service_{"/servos/configure", &RosServoController::configureCallback, this}
  {
    ;
  }


  template<int ...pin_numbers>
  void RosServoController<pin_numbers...>::setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    for(unsigned servo = 0; servo < servo_count_; ++ servo) {
      servos_[servo].setup();
    }
    node_handle_->subscribe(control_subscriber_);
    node_handle_->subscribe(control_raw_subscriber_);
    // Advertise configuration service.
  }


  template<int ...pin_numbers>
  void RosServoController<pin_numbers...>::controlCallback(const ControlServo & message) {
    if(message.servo_id >= 0 && message.servo_id < servo_count_) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      servos_[message.servo_id].setPosition(message.position);
    }
  }


  template<int ...pin_numbers>
  void RosServoController<pin_numbers...>::controlRawCallback(const ControlServoRaw & message) {
    if(message.servo_id >= 0 && message.servo_id < servo_count_) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      servos_[message.servo_id].setPulseWidth(message.pulse_width_us);
    }
  }


  template<int ...pin_numbers>
  void RosServoController<pin_numbers...>::configureCallback(const ConfigureServoRequest & request,
      ConfigureServoResponse & response) {
    ;
  }

}


#endif