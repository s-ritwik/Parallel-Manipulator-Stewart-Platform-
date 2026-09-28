// Stewart platform joint controller for Gazebo Harmonic (gz-sim 8).
//
// Port of the Gazebo Classic ModelPlugin. The ROS 2 interface is unchanged:
//   sub  /stewart/legs_position_cmd  std_msgs/Float32MultiArray  offsets from mid-stroke
//   sub  /stewart/legs_force_cmd     std_msgs/Float32MultiArray  one-shot extra force
//   sub  /stewart/pid_cmd            geometry_msgs/Vector3       P, I, D
//   sub  /stewart/base_pose_cmd      geometry_msgs/Pose          teleport the model
//   pub  /stewart/joint_efforts      std_msgs/Int32MultiArray
//   pub  /stewart/top_platform_pose  geometry_msgs/PoseStamped
//   pub  /stewart/top_platform_vel   geometry_msgs/TwistStamped
//
// Differences from Classic:
// * gz-sim has no JointController, so each piston runs its own gz::math::PID
//   and the output is written as a joint force command every step.
// * DART cannot build closed kinematic chains from SDF joints, so the legs end
//   in "top_cap*_link" links and this plugin welds each of them to the platform
//   at runtime through DetachableJoint components (see convert_to_gz.py).
// * Stamps use simulation time, which is what gazebo_ros used as well.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace stewart_platform
{

class SDFJointController
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  ~SDFJointController() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "[SDFJointController] must be attached to a model, plugin not loaded\n";
      return;
    }
    const std::string model_name = model_.Name(ecm);

    top_link_ = gz::sim::Link(model_.LinkByName(ecm, "platform_link"));
    base_link_ = gz::sim::Link(model_.LinkByName(ecm, "base_link"));
    if (!top_link_.Valid(ecm)) {
      gzerr << "Top platform link 'platform_link' not found in model '" << model_name << "'\n";
    }
    if (!base_link_.Valid(ecm)) {
      gzerr << "Base platform link 'base_link' not found in model '" << model_name << "'\n";
    }

    // Collect only prismatic "piston" joints and sort them by name
    std::vector<std::pair<std::string, gz::sim::Entity>> pistons;
    for (const auto joint_entity : model_.Joints(ecm)) {
      gz::sim::Joint joint(joint_entity);
      const auto name = joint.Name(ecm);
      const auto type = joint.Type(ecm);
      if (name && type && *type == sdf::JointType::PRISMATIC &&
        name->find("piston") != std::string::npos)
      {
        pistons.emplace_back(*name, joint_entity);
      }
    }
    std::sort(pistons.begin(), pistons.end());
    for (const auto & [name, joint_entity] : pistons) {
      joint_names_.push_back(name);
      actuated_joints_.emplace_back(joint_entity);
      actuated_joints_.back().EnablePositionCheck(ecm, true);
    }

    if (actuated_joints_.empty()) {
      gzerr << "[SDFJointController] No prismatic 'piston' joints found.\n";
      return;
    }

    if (top_link_.Valid(ecm)) {
      top_link_.EnableVelocityChecks(ecm, true);
    }

    const auto frame = getString(sdf, "pose_frame", "enu");
    classic_pose_frame_ = (frame == "classic");

    createLoopClosures(ecm);
    initRos();

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] Model '%s': %zu actuated joints, pose_frame=%s",
      model_name.c_str(), actuated_joints_.size(), classic_pose_frame_ ? "classic" : "enu");
    for (const auto & name : joint_names_) {
      RCLCPP_INFO(ros_node_->get_logger(), "  Actuated joint: %s", name.c_str());
    }

    initPidFromSdf(sdf);
    initLegHomeFromLimits(ecm);
    setupRosInterfaces();
    configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }

    std::vector<double> targets;
    std::vector<float> extra_force;
    std::optional<gz::math::Pose3d> base_pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pid_update_) {
        for (auto & pid : pids_) {
          pid = *pid_update_;
        }
        pid_update_.reset();
      }
      targets = targets_;
      extra_force.swap(pending_force_);
      if (has_base_pose_cmd_) {
        base_pose = base_pose_cmd_;
      }
    }

    // Classic moved base_link every step with SetWorldPose.
    if (base_pose) {
      model_.SetWorldPoseCmd(ecm, (*base_pose) * base_offset_.Inverse());
    }

    const auto dt = info.dt;
    for (size_t i = 0; i < actuated_joints_.size(); ++i) {
      const auto pos = actuated_joints_[i].Position(ecm);
      if (!pos || pos->empty()) {
        continue;  // position component is filled after the first physics step
      }
      double force = pids_[i].Update((*pos)[0] - targets[i], dt);
      if (i < extra_force.size()) {
        force += extra_force[i];
      }
      applied_force_[i] = force;
      actuated_joints_[i].SetForce(ecm, {force});
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }

    std_msgs::msg::Int32MultiArray effort_msg;
    effort_msg.data.reserve(applied_force_.size());
    for (const double force : applied_force_) {
      effort_msg.data.push_back(static_cast<int32_t>(force));
    }
    effort_pub_->publish(effort_msg);

    const auto pose = top_link_.WorldPose(ecm);
    if (!pose) {
      return;
    }
    const auto linear_vel = top_link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    const auto angular_vel = top_link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);

    const auto sim_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(sim_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000LL);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "world";
    const auto p = toOutputFrame(pose->Pos());
    pose_msg.pose.position.x = p.X();
    pose_msg.pose.position.y = p.Y();
    pose_msg.pose.position.z = p.Z();
    pose_msg.pose.orientation.x = pose->Rot().X();
    pose_msg.pose.orientation.y = pose->Rot().Y();
    pose_msg.pose.orientation.z = pose->Rot().Z();
    pose_msg.pose.orientation.w = pose->Rot().W();
    top_pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp = stamp;
    vel_msg.header.frame_id = "world";
    const auto v = toOutputFrame(linear_vel);
    const auto w = toOutputFrame(angular_vel);
    vel_msg.twist.linear.x = v.X();
    vel_msg.twist.linear.y = v.Y();
    vel_msg.twist.linear.z = v.Z();
    vel_msg.twist.angular.x = w.X();
    vel_msg.twist.angular.y = w.Y();
    vel_msg.twist.angular.z = w.Z();
    top_vel_pub_->publish(vel_msg);
  }

private:
  // Weld every leg's top cap to the platform (closed-chain loop closure).
  void createLoopClosures(gz::sim::EntityComponentManager & ecm)
  {
    if (!top_link_.Valid(ecm)) {
      return;
    }
    size_t count = 0;
    for (const auto link_entity : model_.Links(ecm)) {
      const auto name = ecm.Component<gz::sim::components::Name>(link_entity);
      if (!name || name->Data().rfind("top_cap", 0) != 0) {
        continue;
      }
      const auto joint = ecm.CreateEntity();
      ecm.CreateComponent(joint, gz::sim::components::DetachableJoint(
          {link_entity, top_link_.Entity(), "fixed"}));
      ecm.CreateComponent(joint, gz::sim::components::ParentEntity(model_.Entity()));
      ++count;
    }
    gzmsg << "[SDFJointController] Welded " << count << " leg top caps to platform_link\n";
  }

  void initRos()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    ros_node_ = std::make_shared<rclcpp::Node>("stewart_joint_controller");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  void initPidFromSdf(const std::shared_ptr<const sdf::Element> & sdf)
  {
    const auto pid_block = (sdf && sdf->HasElement("gazebo_controller_init")) ?
      sdf->FindElement("gazebo_controller_init") : nullptr;

    // Default: RL-tuned gains (can be overridden from SDF)
    const double propor = getValue(pid_block, "propor", 64.0);
    const double integr = getValue(pid_block, "integr", 35.0);
    const double deriv = getValue(pid_block, "deriv", 0.5);
    const double i_max = getValue(pid_block, "i_max", 1.0);
    const double i_min = getValue(pid_block, "i_min", -1.0);
    const double cmd_max = getValue(pid_block, "cmd_max", 500.0);
    const double cmd_min = getValue(pid_block, "cmd_min", -500.0);

    pids_.assign(
      actuated_joints_.size(),
      gz::math::PID(propor, integr, deriv, i_max, i_min, cmd_max, cmd_min));
    applied_force_.assign(actuated_joints_.size(), 0.0);

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] PID: P=%.3f I=%.3f D=%.3f, cmd=[%.2f, %.2f]",
      propor, integr, deriv, cmd_min, cmd_max);
  }

  // Define "home" as mid-stroke (centre between lower and upper limits)
  void initLegHomeFromLimits(const gz::sim::EntityComponentManager & ecm)
  {
    initial_positions_.clear();
    for (size_t i = 0; i < actuated_joints_.size(); ++i) {
      double lower = -std::numeric_limits<double>::infinity();
      double upper = std::numeric_limits<double>::infinity();
      const auto axes = actuated_joints_[i].Axis(ecm);
      if (axes && !axes->empty()) {
        lower = (*axes)[0].Lower();
        upper = (*axes)[0].Upper();
      }
      // Fallback: spawn value (0) if limits are not finite
      const double center = (std::isfinite(lower) && std::isfinite(upper)) ?
        0.5 * (lower + upper) : 0.0;
      initial_positions_.push_back(center);

      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[SDFJointController] Joint %s limits [%.4f, %.4f], center=%.4f",
        joint_names_[i].c_str(), lower, upper, center);
    }
    targets_ = initial_positions_;

    // SetWorldPoseCmd moves the model frame, base_pose_cmd is for base_link.
    if (base_link_.Valid(ecm)) {
      const auto model_pose = gz::sim::worldPose(model_.Entity(), ecm);
      const auto base_pose = base_link_.WorldPose(ecm).value_or(model_pose);
      base_offset_ = model_pose.Inverse() * base_pose;
    }
  }

  void setupRosInterfaces()
  {
    using std::placeholders::_1;

    // Topics are hard-coded to "/stewart/...".
    // Ensure your IK and pose publisher use the same names.
    position_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_position_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePositionCommand, this, _1));

    pid_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/stewart/pid_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePidCommand, this, _1));

    force_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_force_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handleForceCommand, this, _1));

    base_pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Pose>(
      "/stewart/base_pose_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handleBasePoseCommand, this, _1));

    effort_pub_ = ros_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/stewart/joint_efforts", rclcpp::SystemDefaultsQoS());

    top_pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/stewart/top_platform_pose", rclcpp::SystemDefaultsQoS());
    top_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/stewart/top_platform_vel", rclcpp::SystemDefaultsQoS());
  }

  // IK sends offsets from "home" (mid-stroke). We add these to the home positions.
  void handlePositionCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < actuated_joints_.size()) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "[SDFJointController] legs_position_cmd size %zu < num actuated joints %zu",
        msg->data.size(), actuated_joints_.size());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < actuated_joints_.size(); ++i) {
      const float delta = msg->data[i];   // +/- offset from mid-stroke
      targets_[i] = initial_positions_[i] + static_cast<double>(delta);
    }
  }

  void handlePidCommand(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // Same semantics as Classic common::PID(p, i, d): no integral term, no clamp.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pid_update_ = gz::math::PID(msg->x, msg->y, msg->z, 0.0, 0.0, -1.0, 0.0);
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] PID updated from /stewart/pid_cmd: P=%.3f I=%.3f D=%.3f",
      msg->x, msg->y, msg->z);
  }

  void handleForceCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_force_.assign(msg->data.begin(), msg->data.end());
  }

  void handleBasePoseCommand(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    base_pose_cmd_.Pos().Set(msg->position.x, msg->position.y, msg->position.z);
    base_pose_cmd_.Rot() = gz::math::Quaterniond(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    has_base_pose_cmd_ = true;
  }

  // "enu": Harmonic ArduPilot worlds are ENU, same as MAVROS local frame.
  // "classic": the Gazebo Classic mapping (x, y) -> (-y, x).
  gz::math::Vector3d toOutputFrame(const gz::math::Vector3d & v) const
  {
    if (classic_pose_frame_) {
      return {-v.Y(), v.X(), v.Z()};
    }
    return v;
  }

  static double getValue(
    const sdf::ElementConstPtr & parent, const std::string & key, double fallback)
  {
    if (!parent || !parent->HasElement(key)) {
      return fallback;
    }
    return parent->Get<double>(key);
  }

  static std::string getString(
    const std::shared_ptr<const sdf::Element> & parent, const std::string & key,
    const std::string & fallback)
  {
    if (!parent || !parent->HasElement(key)) {
      return fallback;
    }
    return parent->Get<std::string>(key);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link top_link_{gz::sim::kNullEntity};
  gz::sim::Link base_link_{gz::sim::kNullEntity};
  std::vector<gz::sim::Joint> actuated_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> initial_positions_;
  std::vector<gz::math::PID> pids_;
  std::vector<double> applied_force_;
  gz::math::Pose3d base_offset_;
  bool classic_pose_frame_{false};
  bool configured_{false};

  // Shared with ROS callbacks
  std::mutex mutex_;
  std::vector<double> targets_;
  std::vector<float> pending_force_;
  std::optional<gz::math::PID> pid_update_;
  gz::math::Pose3d base_pose_cmd_;
  bool has_base_pose_cmd_{false};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr effort_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr top_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr top_vel_pub_;
};

}  // namespace stewart_platform

GZ_ADD_PLUGIN(
  stewart_platform::SDFJointController,
  gz::sim::System,
  stewart_platform::SDFJointController::ISystemConfigure,
  stewart_platform::SDFJointController::ISystemPreUpdate,
  stewart_platform::SDFJointController::ISystemPostUpdate)
