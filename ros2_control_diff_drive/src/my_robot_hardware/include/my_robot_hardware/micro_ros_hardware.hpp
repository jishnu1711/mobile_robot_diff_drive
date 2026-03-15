#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace my_robot_hardware
{

class MicroRosHardware : public hardware_interface::SystemInterface
{
public:
  // Lifecycle callbacks — called by the controller manager in order.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface registration — tells ros2_control what we expose.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // The 50 Hz control loop callbacks.
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ROS 2 node and communication objects.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;

  // Background executor and its thread.
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  // Shared state between the subscriber callback and the control loop.
  // Protected by state_mutex_.
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  bool data_received_{false};
  std::mutex state_mutex_;

  // Command buffers written by DiffDriveController, read in write().
  // Only accessed from the control loop thread — no mutex needed.
  std::vector<double> joint_velocity_commands_;
};

}  // namespace my_robot_hardware