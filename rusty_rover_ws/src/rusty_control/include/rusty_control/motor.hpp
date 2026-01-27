#pragma once

#ifndef RUSTY_CONTROL__MOTOR_HPP_
#define RUSTY_CONTROL__MOTOR_HPP_

#include <memory>
#include <tuple>

#include "rusty_control/i2c_device.hpp"

using u8 = uint8_t;
using I2CDevicePtr = std::shared_ptr<rusty_control::I2CDevice>;
using MotorPins = std::tuple<u8, u8, u8>;

namespace rusty_control {
class Motor {
 public:
  Motor() = default;
  Motor(I2CDevicePtr i2c, MotorPins pins, std::string name);
  ~Motor();
  bool trySetVelocity(double velocity);

 private:
  I2CDevicePtr i2c_;
  MotorPins pins_;
  std::string name_;
};
}  // namespace rusty_control

#endif  // RUSTY_CONTROL__MOTOR_HPP_