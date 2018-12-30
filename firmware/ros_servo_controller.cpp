#include "ros_servo_controller.hpp"


namespace ros_servo_controller {

  ConfiguredServo::ConfiguredServo(uint8_t pin_number):
    pin_number_{pin_number},
    pulse_width_minimum_{default_pulse_width_minimum_},
    pulse_width_maximum_{default_pulse_width_maximum_},
    enforce_pulse_width_limits_{default_enforce_pulse_width_limits_},
    pulse_width_center_{default_pulse_width_center_},
    pulse_width_initial_{default_pulse_width_initial_},
    pulse_width_current_{pulse_width_initial_},
    servo_{}
  {
    ;
  }


  ConfiguredServo::~ConfiguredServo() {
    servo_.writeMicroseconds(pulse_width_initial_);
    servo_.detach();
  }


  void ConfiguredServo::setup() {
    servo_.attach(pin_number_);
    servo_.writeMicroseconds(pulse_width_initial_);
  }


  void ConfiguredServo::setPosition(int16_t position) {
    if(position <= -1000) {
      pulse_width_current_ = pulse_width_minimum_;
    }
    else if(position == 0) {
      pulse_width_current_ = pulse_width_center_;
    }
    else if(position >= 1000) {
      pulse_width_current_ = pulse_width_maximum_;
    }
    else if(position < 0) {
      pulse_width_current_ = pulse_width_center_ + int32_t(pulse_width_center_ - pulse_width_minimum_) * position / 1000;
    }
    else {  // position > 0)
      pulse_width_current_ = pulse_width_center_ + int32_t(pulse_width_maximum_ - pulse_width_center_) * position / 1000;
    }
    servo_.writeMicroseconds(pulse_width_current_);
  }


  void ConfiguredServo::setPulseWidth(uint16_t pulse_width) {
    pulse_width_current_ = correctPulseWidth(pulse_width);
    servo_.writeMicroseconds(pulse_width_current_);
  }


  void ConfiguredServo::setDefaultConfiguration() {
    pulse_width_minimum_ = default_pulse_width_minimum_;
    pulse_width_maximum_ = default_pulse_width_maximum_;
    enforce_pulse_width_limits_ = default_enforce_pulse_width_limits_;
    pulse_width_center_ = default_pulse_width_center_;
    pulse_width_initial_ = default_pulse_width_initial_;
    if(enforce_pulse_width_limits_) {
      setPulseWidth(pulse_width_current_);
    }
  }


  void ConfiguredServo::setConfiguration(const ServoConfiguration & configuration) {
    pulse_width_minimum_ = configuration.pulse_width_minimum;
    pulse_width_maximum_ = configuration.pulse_width_maximum;
    enforce_pulse_width_limits_ = configuration.enforce_pulse_width_limits;
    pulse_width_center_ = configuration.pulse_width_center;
    pulse_width_initial_ = configuration.pulse_width_initial;
    if(enforce_pulse_width_limits_) {
      setPulseWidth(pulse_width_current_);
    }
  }


  uint16_t ConfiguredServo::correctPulseWidth(uint16_t pulse_width_requested) const {
    if(enforce_pulse_width_limits_) {
      return max(pulse_width_minimum_, min(pulse_width_maximum_, pulse_width_requested));
    }
    else {
      return pulse_width_requested;
    }
  }

}