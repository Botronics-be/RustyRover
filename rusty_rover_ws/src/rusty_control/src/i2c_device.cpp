#include "rusty_control/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <thread>

const uint8_t kDefaultDeviceAddress = 0x60;
const uint8_t kMode1Reg = 0x00;
const uint8_t kMode2Reg = 0x01;
const uint8_t kPrescaleReg = 0xFE;
const uint8_t kPwmReg = 0x06;
const float kReferenceClockSpeed = 25e6;
const float kPwmFrequency = 1600.0;

using namespace std::chrono_literals;
using namespace rusty_control;

I2CDevice::I2CDevice() {
  i2c_fd_ = open("/dev/i2c-1", O_RDWR);
  if (!i2c_fd_) {
    std::__throw_runtime_error("Failed to open I2C interface!");
  }

  // Select the PWM device
  if (!selectDevice())
    std::__throw_runtime_error("Failed to select PWM device!");

  // Reset the PWM device
  if (!reset()) std::__throw_runtime_error("Failed to reset PWM device!");

  // Set the PWM device clock
  if (!setClock()) std::__throw_runtime_error("Failed to set PWM clock!");
}

I2CDevice::~I2CDevice() {
  reset();
  if (i2c_fd_) {
    close(i2c_fd_);
  }
}

bool I2CDevice::enableMotor(uint8_t pin) {
  buf_[0] = kPwmReg + 4 * pin;
  buf_[1] = 0x00;
  buf_[2] = 0x10;
  buf_[3] = 0x00;
  buf_[4] = 0x00;
  return write(i2c_fd_, buf_, 5) == 5;
}

bool I2CDevice::setDutyCycle(uint8_t pin, uint16_t duty_cycle) {
  buf_[0] = kPwmReg + 4 * pin;

  if (duty_cycle == 0xFFFF) {
    // Special case - fully on
    buf_[1] = 0x00;
    buf_[2] = 0x10;
    buf_[3] = 0x00;
    buf_[4] = 0x00;
  } else if (duty_cycle < 0x0010) {
    // Special case - fully off
    buf_[1] = 0x00;
    buf_[2] = 0x00;
    buf_[3] = 0x00;
    buf_[4] = 0x10;
  } else {
    // Shift by 4 to fit 12-bit register
    uint16_t value = duty_cycle >> 4;
    buf_[1] = (uint8_t)0;
    buf_[2] = (uint8_t)0;
    buf_[3] = (uint8_t)(value & 0xFF);
    buf_[4] = (uint8_t)((value >> 8) & 0xFF);
  }

  return write(i2c_fd_, buf_, 5) == 5;
}

bool I2CDevice::writeReg(uint8_t reg, uint8_t data) {
  buf_[0] = reg;
  buf_[1] = data;
  return write(i2c_fd_, buf_, 2) == 2;
}
std::optional<uint8_t> I2CDevice::readReg(uint8_t reg) {
  buf_[0] = reg;
  if (write(i2c_fd_, buf_, 1) != 1) {
    return std::nullopt;
  }
  if (read(i2c_fd_, buf_, 1) != 1) {
    return std::nullopt;
  }
  return buf_[0];
}

bool I2CDevice::setClock() {
  const uint8_t prescale =
      (kReferenceClockSpeed / 4096.0 / kPwmFrequency + 0.5) - 1;
  if (prescale < 3) return false;

  // Read old mode
  const auto old_mode_op = readReg(kMode1Reg);
  if (!old_mode_op.has_value()) return false;
  const uint8_t old_mode = *old_mode_op;

  // Set a new mode/prescaler, then sleep
  const uint8_t new_mode = (old_mode & 0x7F) | 0x10;
  if (!writeReg(kMode1Reg, new_mode)) return false;
  if (!writeReg(kPrescaleReg, prescale)) return false;
  if (!writeReg(kMode1Reg, old_mode)) return false;
  std::this_thread::sleep_for(5ms);

  // Set mode 1 with autoincrement, fix to stop pca9685 from accepting commands
  // at all addresses
  if (!writeReg(kMode1Reg, old_mode | 0xA0)) return false;

  return true;
}

bool I2CDevice::reset() { return writeReg(kMode1Reg, 0x00); }

bool I2CDevice::selectDevice() {
  return ioctl(i2c_fd_, I2C_SLAVE, kDefaultDeviceAddress) >= 0;
}