// Copyright 2023 Australian Centre For Robotics
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

/*
 * Author: Jerome Justin
 */

#ifndef STEER_PID_CONTROLLER__STEER_PID_CONTROLLER_HPP_
#define STEER_PID_CONTROLLER__STEER_PID_CONTROLLER_HPP_

#define _USE_MATH_DEFINES
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "control_msgs/msg/single_dof_state.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "control_msgs/msg/pid_state.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "angles/angles.h"
#include "std_srvs/srv/set_bool.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "steer_pid_controller_parameters.hpp"

namespace steer_pid_controller
{
  class SteerPidController : public controller_interface::ChainableControllerInterface
  {
  public:
    SteerPidController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;
    using ControllerMeasuredStateMsg = control_msgs::msg::MultiDOFCommand;
    using ControllerModeSrvType = std_srvs::srv::SetBool;
    using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

  protected:
    void update_command_interfaces(std::vector<double> &drive_values, std::vector<double> &steer_values);

    // Parameters from ROS for diff_drive_controller
    std::shared_ptr<steer_pid_controller::ParamListener> param_listener_;
    steer_pid_controller::Params params_;

    std::vector<std::string> dof_names_;
    std::vector<std::string> ref_dof_names_;

    size_t dof_;
    std::vector<double> measured_state_values_;

    using PidPtr = std::shared_ptr<control_toolbox::PidROS>;
    std::vector<PidPtr> pids_;

    // Command subscribers and Controller State publisher
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

    rclcpp::Subscription<ControllerMeasuredStateMsg>::SharedPtr measured_state_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerMeasuredStateMsg>> measured_state_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    // override methods from ChainableControllerInterface
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

    // internal methods
    void update_parameters();
    controller_interface::CallbackReturn configure_parameters();

  private:
    // callback for topic interface
    void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  };
} // namespace steer_pid_controller

#endif // STEER_PID_CONTROLLER__STEER_PID_CONTROLLER_HPP_
