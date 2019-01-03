#ifndef __ROS_SERVO_CONTROLLER__
#define __ROS_SERVO_CONTROLLER__


#include <EEPROM.h>
#include "arduino_micro_ros.h"
#include "configured_servo.hpp"
#include <ros_servo_controller/ControlServo.h>
#include <ros_servo_controller/ControlServoRaw.h>
#include <ros_servo_controller/ServoConfiguration.h>
#include <ros_servo_controller/ConfigureServo.h>


namespace ros_servo_controller {

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
      struct StoredConfiguration {
        uint8_t signature_front;
        ConfiguredServo::Configuration configuration;
        uint8_t signature_rear;
      };
      static constexpr uint16_t configuration_base_address_ = 0U;
      static constexpr uint8_t configuration_signature_ = 31U;  // Just a random constant...
    public:  // Member functions.
      RosServoController(ros::NodeHandle * node_handle);
      ~RosServoController() = default;
      void setup();
    private:  // Member functions.
      void controlCallback(const ControlServo & message);
      void controlRawCallback(const ControlServoRaw & message);
      void configureCallback(const ConfigureServo & message);
      bool loadConfigurationFromEeprom(uint8_t servo_id, ConfiguredServo::Configuration & configuration);
      void saveConfigurationToEeprom(uint8_t servo_id, const ConfiguredServo::Configuration & configuration);
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
    for(unsigned servo_id = 0; servo_id < servo_count_; ++ servo_id) {
      ConfiguredServo::Configuration configuration;
      if(loadConfigurationFromEeprom(servo_id, configuration)) {
        servos_[servo_id].setup(configuration);
      }
      else {  // No configuration found, use the default configuration.
        servos_[servo_id].setup();
      }
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
        ConfiguredServo::Configuration configuration;
        if(loadConfigurationFromEeprom(message.servo_id, configuration)) {
          servos_[message.servo_id].setConfiguration(configuration);
        }
        else {  // No configuration found, use the default configuration.
          servos_[message.servo_id].setDefaultConfiguration();
        }
      }
      else if((message.command & ConfigureServo::COMMAND_PARAMETERS) == ConfigureServo::COMMAND_PARAMETERS_NEW) {
        ConfiguredServo::Configuration configuration;
        configuration.pin_number = pin_numbers_[message.servo_id];
        configuration.pulse_width_minimum = message.configuration.pulse_width_minimum;
        configuration.pulse_width_maximum = message.configuration.pulse_width_maximum;
        configuration.enforce_pulse_width_limits = message.configuration.enforce_pulse_width_limits;
        configuration.pulse_width_center = message.configuration.pulse_width_center;
        configuration.pulse_width_initial = message.configuration.pulse_width_initial;
        servos_[message.servo_id].setConfiguration(configuration);
      }
      // Store new configuration parameters in EEPROM if requested.
      if(message.command & ConfigureServo::COMMAND_STORE_IN_EEPROM) {
        auto configuration = servos_[message.servo_id].getConfiguration();
        saveConfigurationToEeprom(message.servo_id, configuration);
      }
    }
  }

  template<int ...pin_numbers>
  bool RosServoController<pin_numbers...>::loadConfigurationFromEeprom(uint8_t servo_id,
      ConfiguredServo::Configuration & configuration) {
    const uint16_t address = configuration_base_address_ + servo_id * sizeof(StoredConfiguration);
    StoredConfiguration stored_configuration;
    EEPROM.get(address, stored_configuration);
    if(stored_configuration.signature_front == configuration_signature_
       && stored_configuration.signature_rear == configuration_signature_
       && stored_configuration.configuration.pin_number == pin_numbers_[servo_id]) {
      configuration.pin_number = stored_configuration.configuration.pin_number;
      configuration.pulse_width_minimum = stored_configuration.configuration.pulse_width_minimum;
      configuration.pulse_width_maximum = stored_configuration.configuration.pulse_width_maximum;
      configuration.enforce_pulse_width_limits = stored_configuration.configuration.enforce_pulse_width_limits;
      configuration.pulse_width_center = stored_configuration.configuration.pulse_width_center;
      configuration.pulse_width_initial = stored_configuration.configuration.pulse_width_initial;
      return true;
    }
    else {
      return false;
    }
  }

  template<int ...pin_numbers>
  void RosServoController<pin_numbers...>::saveConfigurationToEeprom(uint8_t servo_id,
      const ConfiguredServo::Configuration & configuration) {
    const uint16_t address = configuration_base_address_ + servo_id * sizeof(StoredConfiguration);
    StoredConfiguration stored_configuration;
    stored_configuration.signature_front = 0xffU;
    stored_configuration.configuration.pin_number = configuration.pin_number;
    stored_configuration.configuration.pulse_width_minimum = configuration.pulse_width_minimum;
    stored_configuration.configuration.pulse_width_maximum = configuration.pulse_width_maximum;
    stored_configuration.configuration.enforce_pulse_width_limits = configuration.enforce_pulse_width_limits;
    stored_configuration.configuration.pulse_width_center = configuration.pulse_width_center;
    stored_configuration.configuration.pulse_width_initial = configuration.pulse_width_initial;
    stored_configuration.signature_front = 0xffU;
    // Write everything except the signatures to be sure we'll detect corrupted data due to interrupted writes.
    EEPROM.put(address, stored_configuration);
    stored_configuration.signature_front = configuration_signature_;
    stored_configuration.signature_rear = configuration_signature_;
    EEPROM.put(address, stored_configuration);  // This writes the (correct) signatures.
  }

}


#endif