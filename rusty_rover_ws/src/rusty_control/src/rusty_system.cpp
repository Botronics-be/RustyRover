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

#include "rusty_control/rusty_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace rusty_control;

hardware_interface::CallbackReturn RustySystemHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_velocities_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (auto i = 0u; i < info_.joints.size(); i++) {
    auto pin_en_ =
        stoi(info_.hardware_parameters["pin_enable_" + std::to_string(i)]);
    auto pin_pos_ =
        stoi(info_.hardware_parameters["pin_pos_" + std::to_string(i)]);
    auto pin_neg_ =
        stoi(info_.hardware_parameters["pin_neg_" + std::to_string(i)]);
    motor_pin_sets_.emplace_back(std::make_tuple(pin_en_, pin_pos_, pin_neg_));
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RustySystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RustySystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RustySystemHardware"),
                   "Joint '%s' has %zu state interfaces. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RustySystemHardware"),
          "Joint '%s' have '%s' as only state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize I2C and Motors
  try {
    i2c_device_ = std::make_shared<I2CDevice>();
    for (auto i = 0u; i < info.joints.size(); i++) {
      motors_.emplace_back(i2c_device_, motor_pin_sets_[i],
                           info.joints[i].name);
    }
  } catch (std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("RustySystemHardware"),
                 "Error while initializing I2C device or motors: '%s'",
                 ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RustySystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RustySystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RustySystemHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // set 0s as default values
  for (auto i = 0u; i < hw_velocities_.size(); i++) {
    if (std::isnan(hw_velocities_[i])) {
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  bool success = true;
  for (auto motor : motors_) {
    success = success && motor.setVelocity(0);
  }
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("RustySystemHardware"),
                 "Error setting velocity on motors while activating!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RustySystemHardware"),
              "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RustySystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  bool success = true;
  for (auto motor : motors_) {
    success = success && motor.setVelocity(0);
  }
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("RustySystemHardware"),
                 "Error setting velocity on motors while deactivating!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RustySystemHardware"),
              "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RustySystemHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RustySystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  bool success = true;
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    success = success && motors_[i].setVelocity(hw_commands_[i]);
  }
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("RustySystemHardware"),
                 "Error setting velocity on motors during write step!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rusty_control::RustySystemHardware,
                       hardware_interface::SystemInterface)