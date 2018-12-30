#ifndef __ROS_SERVO_CONTROLLER__
#define __ROS_SERVO_CONTROLLER__


#include "arduino_micro_ros.h"
#include <Servo.h>
#include <ros_servo_controller/ControlServo.h>
#include <ros_servo_controller/ControlServoRaw.h>
#include <ros_servo_controller/ServoConfiguration.h>
#include <ros_servo_controller/ConfigureServo.h>


namespace ros_servo_controller {

  class ConfiguredServo {
    private:
      static constexpr uint16_t default_pulse_width_minimum_{1000U};
      static constexpr uint16_t default_pulse_width_maximum_{2000U};
      static constexpr bool default_enforce_pulse_width_limits_{true};
      static constexpr uint16_t default_pulse_width_center_{1500U};
      static constexpr uint16_t default_pulse_width_initial_{1500U};
      uint8_t pin_number_;
      Servo servo_;
      uint16_t pulse_width_minimum_;
      uint16_t pulse_width_maximum_;
      bool enforce_pulse_width_limits_;
      uint16_t pulse_width_center_;
      uint16_t pulse_width_initial_;
      uint16_t pulse_width_current_;
    public:
      ConfiguredServo(uint8_t pin_number);
      ~ConfiguredServo();
      void setup();
      void setPosition(int16_t position);
      void setPulseWidth(uint16_t pulse_width);
      void setDefaultConfiguration();
      void setConfiguration(const ServoConfiguration & configuration);
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
      ros::Subscriber<ControlServoRaw, RosServoController> control_raw_subscriber_;
      ros::Subscriber<ConfigureServo, RosServoController> configure_subscriber_;
    public:  // Member functions.
      RosServoController(ros::NodeHandle * node_handle);
      ~RosServoController() = default;
      void setup();
    private:  // Member functions.
      void controlCallback(const ControlServo & message);
      void controlRawCallback(const ControlServoRaw & message);
      void configureCallback(const ConfigureServo & message);
  };


  template<int ...pin_numbers>
  RosServoController<pin_numbers...>::RosServoController(ros::NodeHandle * node_handle):
    servo_count_{sizeof...(pin_numbers)},
    pin_numbers_{pin_numbers...},
    servos_{pin_numbers...},
    node_handle_{node_handle},
    control_subscriber_{"/servo/control", &RosServoController::controlCallback, this},
    control_raw_subscriber_{"/servo/control_raw", &RosServoController<pin_numbers...>::controlRawCallback, this},
    configure_subscriber_{"/servo/configure", &RosServoController::configureCallback, this}
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
    node_handle_->subscribe(configure_subscriber_);
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
  void RosServoController<pin_numbers...>::configureCallback(const ConfigureServo & message) {
    if(message.servo_id >= 0 && message.servo_id < servo_count_) {
      // Use requested configuration.
      if((message.command & ConfigureServo::COMMAND_PARAMETERS) == ConfigureServo::COMMAND_PARAMETERS_DEFAULT) {
        servos_[message.servo_id].setDefaultConfiguration();
      }
      else if((message.command & ConfigureServo::COMMAND_PARAMETERS) == ConfigureServo::COMMAND_PARAMETERS_EEPROM) {
        ;  // TODO: get configuration from EEPROM.
      }
      else if((message.command & ConfigureServo::COMMAND_PARAMETERS) == ConfigureServo::COMMAND_PARAMETERS_NEW) {
        servos_[message.servo_id].setConfiguration(message.configuration);
      }
      // Store new configuration parameters in EEPROM if requested.
      if(message.command & ConfigureServo::COMMAND_STORE_IN_EEPROM) {
        ;  // TODO: store configuration in EEPROM.
      }
    }
  }

}


#endif