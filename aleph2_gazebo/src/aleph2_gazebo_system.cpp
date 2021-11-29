#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/node.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "gazebo_ros2_control/gazebo_system_interface.hpp"

namespace aleph2_gazebo
{

enum class JointControlMethod
{
  NONE,
  POSITION,
  VELOCITY,
  EFFORT,
};

class Aleph2GazeboSystem : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  Aleph2GazeboSystem()
  : logger_(rclcpp::get_logger("")) {}

  bool initSim(
    rclcpp::Node::SharedPtr & ros_node,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Logger logger_;

  size_t n_joints_;

  gazebo::physics::ModelPtr parent_model_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  std::vector<JointControlMethod> joint_control_method_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  std::vector<double> joint_position_cmd_;
  std::vector<double> joint_velocity_cmd_;
  std::vector<double> joint_effort_cmd_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

};

bool Aleph2GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & ros_node,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  ros_node_ = ros_node;
  parent_model_ = parent_model;
  logger_ = ros_node_->get_logger().get_child(hardware_info.name);

  return true;
}

CallbackReturn Aleph2GazeboSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(logger_, "No physics engine configured in Gazebo.");
    return CallbackReturn::ERROR;
  }

  n_joints_ = hardware_info.joints.size();
  sim_joints_.resize(n_joints_);
  joint_names_.resize(n_joints_);
  joint_position_.resize(n_joints_);
  joint_velocity_.resize(n_joints_);
  joint_effort_.resize(n_joints_);
  joint_position_cmd_.resize(n_joints_);
  joint_velocity_cmd_.resize(n_joints_);
  joint_effort_cmd_.resize(n_joints_);
  joint_control_method_.resize(n_joints_, JointControlMethod::NONE);

  for (unsigned int j = 0; j < n_joints_; j++) {
    std::string joint_name = joint_names_[j] = hardware_info.joints[j].name;

    gazebo::physics::JointPtr simjoint = parent_model_->GetJoint(joint_name);
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        logger_, "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim_joints_[j] = simjoint;

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(logger_, "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(logger_, "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_INFO_STREAM(logger_, "\t\t position");
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_INFO_STREAM(logger_, "\t\t velocity");
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
        hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_INFO_STREAM(logger_, "\t\t effort");
        command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &joint_effort_cmd_[j]);
      }
    }

    RCLCPP_INFO_STREAM(logger_, "\tState:");

    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++) {
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO_STREAM(logger_, "\t\t position");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &joint_position_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO_STREAM(logger_, "\t\t velocity");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &joint_velocity_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO_STREAM(logger_, "\t\t effort");
        state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &joint_effort_[j]);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Aleph2GazeboSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> Aleph2GazeboSystem::export_command_interfaces()
{
  return std::move(command_interfaces_);
}

hardware_interface::return_type Aleph2GazeboSystem::read()
{
  for (unsigned int j = 0; j < joint_names_.size(); j++) {
    if (sim_joints_[j]) {
      joint_position_[j] = sim_joints_[j]->Position(0);
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce(0);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Aleph2GazeboSystem::write()
{
  for (unsigned int j = 0; j < joint_names_.size(); j++) {
    if (sim_joints_[j]) {
      switch (joint_control_method_[j]) {
        case JointControlMethod::POSITION:
          sim_joints_[j]->SetPosition(0, joint_position_cmd_[j], true);
          break;
        case JointControlMethod::VELOCITY:
          sim_joints_[j]->SetVelocity(0, joint_velocity_cmd_[j]);
          break;
        case JointControlMethod::EFFORT:
          sim_joints_[j]->SetForce(0, joint_effort_cmd_[j]);
          break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace aleph2_gazebo

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  aleph2_gazebo::Aleph2GazeboSystem,
  gazebo_ros2_control::GazeboSystemInterface)
