#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace gazebo
{
class SDFJointController : public ModelPlugin
{
public:
  SDFJointController() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    if (!model) {
      gzerr << "Invalid model pointer, plugin not loaded\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "Failed to get gazebo_ros node, plugin not loaded\n";
      return;
    }

    model_ = model;
    joints_ = model_->GetJoints();
    for (const auto & joint : joints_) {
      if (joint->GetMsgType() == gazebo::msgs::Joint::PRISMATIC) {
        actuated_joints_.push_back(joint);
      }
    }
    if (actuated_joints_.empty()) {
      gzerr << "No prismatic joints found for Stewart controller.\n";
      return;
    }

    RCLCPP_INFO(ros_node_->get_logger(),
      "Custom joint controller attached to model '%s' with %zu total joints (%zu actuated)",
      model_->GetName().c_str(), joints_.size(), actuated_joints_.size());

    initPidFromSdf(sdf);
    initLegLengthsFromSdf(sdf);
    setupRosInterfaces();
  }

private:
  void initPidFromSdf(const sdf::ElementPtr & sdf)
  {
    const sdf::ElementPtr pid_block = sdf->HasElement("gazebo_controller_init")
      ? sdf->GetElement("gazebo_controller_init")
      : nullptr;

    propor_ = getValue(pid_block, "propor", 64.0);
    integr_ = getValue(pid_block, "integr", 35.0);
    deriv_ = getValue(pid_block, "deriv", 0.5);

    pid_ = common::PID(propor_, integr_, deriv_);

    auto controller = model_->GetJointController();
    for (const auto & joint : actuated_joints_) {
      controller->SetPositionPID(joint->GetScopedName(), pid_);
    }

    RCLCPP_INFO(ros_node_->get_logger(),
      "Initial PID gains set to P=%.3f I=%.3f D=%.3f", propor_, integr_, deriv_);
  }

  void initLegLengthsFromSdf(const sdf::ElementPtr & sdf)
  {
    const sdf::ElementPtr pid_block = sdf->HasElement("gazebo_controller_init")
      ? sdf->GetElement("gazebo_controller_init")
      : nullptr;

    const double init_length = getValue(pid_block, "init_legs_length", 0.3);

    auto controller = model_->GetJointController();
    for (const auto & joint : actuated_joints_) {
      controller->SetPositionTarget(joint->GetScopedName(), init_length);
    }
  }

  void setupRosInterfaces()
  {
    using std::placeholders::_1;

    position_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/" + model_->GetName() + "/legs_position_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePositionCommand, this, _1));

    pid_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/" + model_->GetName() + "/pid_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePidCommand, this, _1));

    force_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/" + model_->GetName() + "/legs_force_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handleForceCommand, this, _1));

    effort_pub_ = ros_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/" + model_->GetName() + "/joint_efforts", rclcpp::SystemDefaultsQoS());

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SDFJointController::onUpdate, this, std::placeholders::_1));

  }

  void handlePositionCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }

    auto controller = model_->GetJointController();
    auto joint_it = actuated_joints_.begin();
    for (const float target : msg->data) {
      if (joint_it == actuated_joints_.end()) {
        break;
      }
      controller->SetPositionTarget((*joint_it)->GetScopedName(), target);
      ++joint_it;
    }
  }

  void handlePidCommand(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    pid_ = common::PID(msg->x, msg->y, msg->z);
    auto controller = model_->GetJointController();
    for (const auto & joint : actuated_joints_) {
      controller->SetPositionPID(joint->GetScopedName(), pid_);
    }
  }

  void handleForceCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    auto joint_it = actuated_joints_.begin();
    for (const float force : msg->data) {
      if (joint_it == actuated_joints_.end()) {
        break;
      }
      (*joint_it)->SetForce(0, force);
      ++joint_it;
    }
  }

  void onUpdate(const common::UpdateInfo &)
  {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(actuated_joints_.size());
    for (const auto & joint : actuated_joints_) {
      msg.data.push_back(static_cast<int32_t>(joint->GetForce(0)));
    }
    effort_pub_->publish(msg);
  }

  static double getValue(const sdf::ElementPtr & parent, const std::string & key, double fallback)
  {
    if (!parent || !parent->HasElement(key)) {
      return fallback;
    }
    return parent->Get<double>(key);
  }

  physics::ModelPtr model_;
  std::vector<physics::JointPtr> joints_;
  std::vector<physics::JointPtr> actuated_joints_;

  std::shared_ptr<gazebo_ros::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr effort_pub_;

  event::ConnectionPtr update_connection_;

  double propor_{64.0};
  double integr_{35.0};
  double deriv_{0.5};
  common::PID pid_;
};

GZ_REGISTER_MODEL_PLUGIN(SDFJointController)
}  // namespace gazebo
