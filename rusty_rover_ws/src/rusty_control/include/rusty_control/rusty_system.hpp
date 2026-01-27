// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file was last modified by Michael Hart, a.k.a Mike Likes Robots
// (mikelikesrobots@outlook.com), on 2024-03-07.

#ifndef RUSTY_CONTROL__RUSTY_SYSTEM_HPP_
#define RUSTY_CONTROL__RUSTY_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rusty_control/i2c_device.hpp"
#include "rusty_control/motor.hpp"
#include "rusty_control/visibility_control.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rusty_control {
class RustySystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RustySystemHardware)

  RUSTY_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  RUSTY_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  RUSTY_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  RUSTY_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  RUSTY_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  RUSTY_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  RUSTY_CONTROL_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::vector<MotorPins> motor_pin_sets_;
  std::vector<Motor> motors_;
  std::shared_ptr<I2CDevice> i2c_device_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
};

}  // namespace rusty_control

#endif  // RUSTY_CONTROL__RUSTY_SYSTEM_HPP_