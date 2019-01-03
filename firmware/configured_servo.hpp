#ifndef __CONFIGURED_SERVO__
#define __CONFIGURED_SERVO__


#include <Servo.h>


namespace ros_servo_controller {

  class ConfiguredServo {
    public:
      struct Configuration {
        uint8_t pin_number;
        uint16_t pulse_width_minimum;
        uint16_t pulse_width_maximum;
        bool enforce_pulse_width_limits;
        uint16_t pulse_width_center;
        uint16_t pulse_width_initial;
      };
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
      void setup(const Configuration & configuration);
      void setPosition(int16_t position);
      void setPulseWidth(uint16_t pulse_width);
      void setDefaultConfiguration();
      void setConfiguration(const Configuration & configuration);
      Configuration getConfiguration();
    private:
      uint16_t correctPulseWidth(uint16_t pulse_width_requested) const;
  };

}


#endif