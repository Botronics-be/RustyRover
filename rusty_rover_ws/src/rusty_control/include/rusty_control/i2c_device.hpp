#ifndef RUSTY_CONTROL__I2C_DEVICE_HPP_
#define RUSTY_CONTROL__I2C_DEVICE_HPP_

#include <cstdint>
#include <mutex>
#include <optional>

namespace rusty_control {

class I2CDevice {
 public:
  I2CDevice();
  ~I2CDevice();
  bool enableMotor(uint8_t pin);
  bool setDutyCycle(uint8_t pin, uint16_t duty_cycle);

 private:
  bool selectDevice();
  bool writeReg(uint8_t reg, uint8_t data);
  std::optional<uint8_t> readReg(uint8_t reg);
  bool setClock();
  bool reset();
  int i2c_fd_;
  uint8_t buf_[10];
};

}  // namespace rusty_control

#endif  // RUSTY_CONTROL__I2C_DEVICE_HPP_