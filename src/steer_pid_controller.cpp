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

#include "steer_pid_controller/steer_pid_controller.hpp"

namespace
{  // utility
// Changed services history QoS to keep all so we don't lose any client service calls
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 17
rclcpp::QoS qos_services =
    rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1)).reliable().durability_volatile();
#else
static const rmw_qos_profile_t qos_services = { RMW_QOS_POLICY_HISTORY_KEEP_ALL,
                                                1,  // message queue depth
                                                RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                RMW_QOS_DEADLINE_DEFAULT,
                                                RMW_QOS_LIFESPAN_DEFAULT,
                                                RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                false };
#endif

using ControllerCommandMsg = steer_pid_controller::SteerPidController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(const std::shared_ptr<ControllerCommandMsg>& msg,
                                    const std::vector<std::string>& ref_dof_names)
{
  msg->dof_names = ref_dof_names;
  msg->values.resize(ref_dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg(const std::shared_ptr<ControllerCommandMsg>& msg,
                                         const std::vector<std::string>& dof_names)
{
  reset_controller_reference_msg(msg, dof_names);
}

}  // namespace

namespace steer_pid_controller
{
SteerPidController::SteerPidController() : controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn SteerPidController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<steer_pid_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void SteerPidController::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn SteerPidController::configure_parameters()
{
  update_parameters();

  dof_names_ = params_.dof_names;
  ref_dof_names_ = params_.reference_dof_names;

  dof_ = params_.dof_names.size();

  if (params_.gains.dof_names_map.size() != dof_)
  {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Size of 'gains' (%zu) map and number or 'dof_names' (%zu) have to be the same!",
                 params_.gains.dof_names_map.size(), dof_);
    return CallbackReturn::FAILURE;
  }

  pids_.resize(dof_);

  for (size_t i = 0; i < dof_; ++i)
  {
    // prefix should be interpreted as parameters prefix
    pids_[i] = std::make_shared<control_toolbox::PidROS>(get_node(), "gains." + params_.dof_names[i], true);
    if (!pids_[i]->initPid())
    {
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteerPidController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  dof_names_.clear();
  ref_dof_names_.clear();

  pids_.clear();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteerPidController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
      "~/reference", subscribers_qos, std::bind(&SteerPidController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, ref_dof_names_);
  input_ref_.writeFromNonRT(msg);

  std::shared_ptr<ControllerMeasuredStateMsg> measured_state_msg = std::make_shared<ControllerMeasuredStateMsg>();
  reset_controller_measured_state_msg(measured_state_msg, dof_names_);
  measured_state_.writeFromNonRT(measured_state_msg);

  measured_state_values_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher
  state_publisher_->lock();
  state_publisher_->msg_.dof_states.resize(dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    state_publisher_->msg_.dof_states[i].name = dof_names_[i];
  }
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void SteerPidController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->dof_names.empty() && msg->values.size() == ref_dof_names_.size())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Reference massage does not have DoF names defined. "
                                          "Assuming that value have order as defined state DoFs");
    auto ref_msg = msg;
    ref_msg->dof_names = ref_dof_names_;
    input_ref_.writeFromNonRT(ref_msg);
  }
  else if (msg->dof_names.size() == ref_dof_names_.size() && msg->values.size() == ref_dof_names_.size())
  {
    auto ref_msg = msg;  // simple initialization

    // sort values in the ref_msg
    reset_controller_reference_msg(msg, ref_dof_names_);

    bool all_found = true;
    for (size_t i = 0; i < msg->dof_names.size(); ++i)
    {
      auto found_it = std::find(ref_msg->dof_names.begin(), ref_msg->dof_names.end(), msg->dof_names[i]);
      if (found_it == msg->dof_names.end())
      {
        all_found = false;
        RCLCPP_WARN(get_node()->get_logger(), "DoF name '%s' not found in the defined list of state DoFs.",
                    msg->dof_names[i].c_str());
        break;
      }

      auto position = std::distance(ref_msg->dof_names.begin(), found_it);
      ref_msg->values[position] = msg->values[i];
      ref_msg->values_dot[position] = msg->values_dot[i];
    }

    if (all_found)
    {
      input_ref_.writeFromNonRT(ref_msg);
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Size of input data names (%zu) and/or values (%zu) is not matching the expected size (%zu).",
                 msg->dof_names.size(), msg->values.size(), ref_dof_names_.size());
  }
}

controller_interface::InterfaceConfiguration SteerPidController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.dof_names.size());
  for (const auto& dof_name : params_.dof_names)
  {
    command_interfaces_config.names.push_back(dof_name + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SteerPidController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(dof_ * 2);

  for (const auto& dof_name : dof_names_)
  {
    state_interfaces_config.names.push_back(dof_name + "/position");
  }

  for (const auto& dof_name : dof_names_)
  {
    state_interfaces_config.names.push_back(dof_name + "/velocity");
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> SteerPidController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(dof_);

  auto logger = get_node()->get_logger();

  for (std::size_t i = 0; i < ref_dof_names_.size(); i++)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), ref_dof_names_[i] + "/" + hardware_interface::HW_IF_POSITION,
        &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool SteerPidController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn SteerPidController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_controller_reference_msg(*(input_ref_.readFromRT()), ref_dof_names_);

  reset_controller_measured_state_msg(*(measured_state_.readFromRT()), dof_names_);

  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  measured_state_values_.assign(measured_state_values_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteerPidController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
SteerPidController::update_reference_from_subscribers(const rclcpp::Time& time, const rclcpp::Duration& /*period*/
)
{
  auto current_ref = input_ref_.readFromRT();

  for (size_t i = 0; i < dof_; ++i)
  {
    if (!std::isnan((*current_ref)->values[i]))
    {
      reference_interfaces_[i] = (*current_ref)->values[i];
      (*current_ref)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type SteerPidController::update_and_write_commands(const rclcpp::Time& time,
                                                                                const rclcpp::Duration& period)
{
  // check for any parameter updates
  update_parameters();

  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    measured_state_values_[i] = state_interfaces_[i].get_value();
  }

  for (size_t i = 0; i < dof_; ++i)
  {
    double steer_control_position_output = std::numeric_limits<double>::quiet_NaN();
    double steer_control_velocity_output = std::numeric_limits<double>::quiet_NaN();

    if (!std::isnan(reference_interfaces_[i]) && !std::isnan(measured_state_values_[i]))
    {
      double position_setpoint = reference_interfaces_[i];
      double process_position = measured_state_values_[i];
      double process_velocity = state_interfaces_[dof_ + i].get_value();

      steer_control_position_output = 0.0;
      steer_control_velocity_output = 0.0;

      double position_error = 0.0;
      double velocity_error = 0.0;

      if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
      {
        // for continuous angles the error is normalized between -pi<error<pi
        position_error = angles::shortest_angular_distance(process_position, position_setpoint);
      }
      else
      {
        position_error = position_setpoint - process_position;
      }

      steer_control_position_output += pids_[i]->computeCommand(position_error, period);

      double velocity_setpoint = steer_control_position_output;
      velocity_error = velocity_setpoint - process_velocity;

      steer_control_velocity_output += pids_[i]->computeCommand(velocity_error, period);

      // write calculated values
      command_interfaces_[i].set_value(steer_control_velocity_output);
    }
  }

  // if (state_publisher_ && state_publisher_->trylock())
  // {
  //   state_publisher_->msg_.header.stamp = time;
  //   for (size_t i = 0; i < dof_; ++i)
  //   {
  //     state_publisher_->msg_.dof_states[i].reference = reference_interfaces_[i];
  //     state_publisher_->msg_.dof_states[i].feedback = measured_state_values_[i];

  //     double steer_control_position_output = 0.0;
  //     double position_error = 0.0;
  //     double velocity_error = 0.0;

  //     if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
  //     {
  //       // for continuous angles the error is normalized between -pi<error<pi
  //       position_error = angles::shortest_angular_distance(measured_state_values_[i], reference_interfaces_[i]);
  //     }
  //     else
  //     {
  //       position_error = reference_interfaces_[i] - measured_state_values_[i];
  //     }

  //     steer_control_position_output += pids_[i]->computeCommand(position_error, period);

  //     double velocity_setpoint = steer_control_position_output;
  //     velocity_error = velocity_setpoint - state_interfaces_[dof_ + i].get_value();

  //     state_publisher_->msg_.dof_states[i].error = position_error;
  //     state_publisher_->msg_.dof_states[i].error_dot = velocity_error;

  //     state_publisher_->msg_.dof_states[i].time_step = period.seconds();
  //     // Command can store the old calculated values. This should be obvious because at least one
  //     // another value is NaN.
  //     state_publisher_->msg_.dof_states[i].output = command_interfaces_[i].get_value();
  //   }
  //   state_publisher_->unlockAndPublish();
  // }

  return controller_interface::return_type::OK;
}

}  // namespace steer_pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(steer_pid_controller::SteerPidController, controller_interface::ChainableControllerInterface)
