#include "ros_servo_controller.hpp"


namespace ros_servo_controller {

  ConfiguredServo::ConfiguredServo(uint8_t pin_number):
    pin_number_{pin_number},
    pulse_width_minimum_{1000U},
    pulse_width_maximum_{2000U},
    pulse_width_enforce_limits_{false},
    pulse_width_center_{correctPulseWidth(1500U)},
    pulse_width_initial_{correctPulseWidth(1500U)},
    servo_{}
  {
    ;
  }


  ConfiguredServo::~ConfiguredServo() {
    servo_.writeMicroseconds(pulse_width_initial_);
    servo_.detach();
  }


  void ConfiguredServo::setup() {
    servo_.attach(pin_number_, pulse_width_minimum_, pulse_width_maximum_);
    servo_.writeMicroseconds(pulse_width_initial_);
  }


  void ConfiguredServo::setPosition(int16_t position) {
    if(position <= -1000) {
      servo_.writeMicroseconds(pulse_width_minimum_);
    }
    else if(position == 0) {
      servo_.writeMicroseconds(pulse_width_center_);
    }
    else if(position >= 1000) {
      servo_.writeMicroseconds(pulse_width_maximum_);
    }
    else if(position < 0) {
      servo_.writeMicroseconds(pulse_width_center_ + int32_t(pulse_width_center_ - pulse_width_minimum_) * position / 1000);
    }
    else {  // position > 0)
      servo_.writeMicroseconds(pulse_width_center_ + int32_t(pulse_width_maximum_ - pulse_width_center_) * position / 1000);
    }
  }


  void ConfiguredServo::setPulseWidth(uint16_t pulse_width) {
    servo_.writeMicroseconds(correctPulseWidth(pulse_width));
  }


  uint16_t ConfiguredServo::correctPulseWidth(uint16_t pulse_width_requested) const {
    if(pulse_width_enforce_limits_) {
      return max(pulse_width_minimum_, min(pulse_width_maximum_, pulse_width_requested));
    }
    else {
      return pulse_width_requested;
    }
  }

}