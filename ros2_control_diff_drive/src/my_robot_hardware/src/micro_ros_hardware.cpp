#include "my_robot_hardware/micro_ros_hardware.hpp"

#include <chrono>
#include <limits>
#include <mutex>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace my_robot_hardware
{

hardware_interface::CallbackReturn MicroRosHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("MicroRosHardware"),
      "Expected 2 joints, got %zu. Check the URDF.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MicroRosHardware"),
        "Joint '%s' must have exactly one velocity command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("MicroRosHardware"),
        "Joint '%s' must have exactly two state interfaces (position + velocity).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  joint_positions_.assign(2, 0.0);
  joint_velocities_.assign(2, 0.0);
  joint_velocity_commands_.assign(2, 0.0);

  // Log joint order so we can verify left=0, right=1 at startup
  RCLCPP_INFO(
    rclcpp::get_logger("MicroRosHardware"),
    "Joint order: [0]=%s  [1]=%s",
    info_.joints[0].name.c_str(),
    info_.joints[1].name.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MicroRosHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MicroRosHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn MicroRosHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MicroRosHardware"), "Activating hardware interface...");

  node_ = std::make_shared<rclcpp::Node>("micro_ros_hardware_node");

  // Subscribe to /esp32/joint_states — avoids conflict with
  // joint_state_broadcaster which also publishes to /joint_states.
  // The ESP32 publishes joint names ["left_wheel", "right_wheel"].
  // We look up by name so joint order in the URDF doesn't matter.
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/esp32/joint_states",
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);

      // Map ESP32 joint names to URDF joint indices by name lookup.
      // URDF joints: info_.joints[0].name, info_.joints[1].name
      // ESP32 names: "left_wheel", "right_wheel"
      // We match by checking which URDF joint contains "left" or "right".
      for (size_t esp_idx = 0; esp_idx < msg->name.size(); ++esp_idx) {
        const std::string & esp_name = msg->name[esp_idx];

        for (size_t urdf_idx = 0; urdf_idx < info_.joints.size(); ++urdf_idx) {
          const std::string & urdf_name = info_.joints[urdf_idx].name;

          // Match "left_wheel" → "base_left_wheel_joint" and
          //       "right_wheel" → "base_right_wheel_joint"
          bool is_left  = (esp_name.find("left")  != std::string::npos &&
                           urdf_name.find("left")  != std::string::npos);
          bool is_right = (esp_name.find("right") != std::string::npos &&
                           urdf_name.find("right") != std::string::npos);

          if (is_left || is_right) {
            if (esp_idx < msg->position.size()) {
              joint_positions_[urdf_idx] = msg->position[esp_idx];
            }
            if (esp_idx < msg->velocity.size()) {
              joint_velocities_[urdf_idx] = msg->velocity[esp_idx];
            }
            break;
          }
        }
      }
      data_received_ = true;
    });

  // Publish per-wheel velocity commands directly as [left_rad_s, right_rad_s].
  // No inverse kinematics here — DiffDriveController already computed these.
  wheel_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/wheel_vel_cmd",
    rclcpp::SystemDefaultsQoS());

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("MicroRosHardware"), "Hardware interface active.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MicroRosHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MicroRosHardware"), "Deactivating hardware interface...");

  // Send zero velocity to both wheels before teardown.
  std_msgs::msg::Float64MultiArray stop_msg;
  stop_msg.data = {0.0, 0.0};
  wheel_vel_pub_->publish(stop_msg);

  if (executor_) { executor_->cancel(); }
  if (spin_thread_.joinable()) { spin_thread_.join(); }

  node_.reset();
  executor_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroRosHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // joint_positions_ and joint_velocities_ are written directly by the
  // subscriber callback — nothing to copy here. Mutex prevents torn reads.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MicroRosHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // joint_velocity_commands_[0] = left  wheel rad/s  (set by DiffDriveController)
  // joint_velocity_commands_[1] = right wheel rad/s  (set by DiffDriveController)
  // No kinematics needed — send directly to ESP32.
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {joint_velocity_commands_[0], joint_velocity_commands_[1]};
  wheel_vel_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MicroRosHardware,
  hardware_interface::SystemInterface)