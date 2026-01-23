#include "rusty_control/motor.hpp"

using namespace rusty_control;

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

Motor::Motor(I2CDevicePtr i2c, MotorPins pins, std::string name)
    : i2c_{i2c}, pins_{pins}, name_{name} {
  u8 enable_pin = std::get<0>(pins_);
  if (!i2c_->tryEnableMotor(enable_pin)) {
    std::string error = "Failed to enable motor " + name_ + "!";
    std::__throw_runtime_error(error.c_str());
  }
}

Motor::~Motor() { trySetVelocity(0); }

bool Motor::trySetVelocity(double velocity) {
  u8 pos_pin = std::get<1>(pins_);
  u8 neg_pin = std::get<2>(pins_);

  // Interpret <1/1000 max speed as not spinning
  if (std::fabs(velocity) < 0.001) {
    if (!i2c_->trySetDutyCycle(pos_pin, 0xFFFF)) {
      return false;
    }
    if (!i2c_->trySetDutyCycle(neg_pin, 0xFFFF)) {
      return false;
    }
    return true;
  }

  uint16_t duty_cycle = 0xFFFF * std::fabs(velocity);

  // Use braking mode
  if (velocity > 0) {
    if (!i2c_->trySetDutyCycle(pos_pin, 0xFFFF)) {
      return false;
    }
    if (!i2c_->trySetDutyCycle(neg_pin, 0xFFFF - duty_cycle)) {
      return false;
    }
  } else {
    if (!i2c_->trySetDutyCycle(pos_pin, 0xFFFF - duty_cycle)) {
      return false;
    }
    if (!i2c_->trySetDutyCycle(neg_pin, 0xFFFF)) {
      return false;
    }
  }

  return true;
}