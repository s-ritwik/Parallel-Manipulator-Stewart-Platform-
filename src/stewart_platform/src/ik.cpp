#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace stewart_platform
{

class InverseKinematicsNode : public rclcpp::Node
{
public:
  InverseKinematicsNode()
  : rclcpp::Node("ik")
  {
    RCLCPP_INFO(this->get_logger(), "[IK] Node constructor called");

    // World-frame z of platform when legs are at mid-stroke
    height_ = static_cast<float>(
      this->declare_parameter("platform_home_height", 2.0625));
    RCLCPP_INFO(
      this->get_logger(),
      "[IK] platform_home_height = %.4f (world z at mid-stroke)", height_);

    initAttachmentPoints();
    initMessage();
    computeRestLengths();

    legs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_position_cmd", rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(this->get_logger(), "[IK] Publisher: /stewart/legs_position_cmd");

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/stewart/platform_pose", rclcpp::SensorDataQoS(),
      std::bind(&InverseKinematicsNode::handlePlatformPose, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "[IK] Subscriber: /stewart/platform_pose");

    RCLCPP_INFO(this->get_logger(), "Inverse kinematics node ready");
  }

private:
  void initAttachmentPoints()
  {
    // Base attachment points in world frame (homogeneous coordinates)
    base_points_ << 1.5588457268119897f, 0.9f, 0.25f, 1.0f,
                    1.1021821192326179e-16f, 1.8f, 0.25f, 1.0f,
                   -1.5588457268119897f, 0.9f, 0.25f, 1.0f,
                   -1.5588457268119895f, -0.9f, 0.25f, 1.0f,
                   -3.3065463576978537e-16f, -1.8f, 0.25f, 1.0f,
                    1.558845726811989f, -0.9000000000000008f, 0.25f, 1.0f;

    // Platform attachment points in platform frame (homogeneous)
    platform_points_ << 0.9545941546018393f, 0.9545941546018392f, -0.05f, 1.0f,
                        0.34940571088840333f, 1.303999865490242f, -0.05f, 1.0f,
                       -1.303999865490242f, 0.3494057108884034f, -0.05f, 1.0f,
                       -1.3039998654902425f, -0.3494057108884025f, -0.05f, 1.0f,
                        0.34940571088840355f, -1.303999865490242f, -0.05f, 1.0f,
                        0.9545941546018399f, -0.9545941546018385f, -0.05f, 1.0f;

    RCLCPP_INFO(this->get_logger(), "[IK] Attachment points initialized");
  }

  void initMessage()
  {
    legs_msg_.data.assign(6, 0.0F);
  }

  // Compute leg lengths at "home" configuration (x=y=0, roll=pitch=yaw=0, z = height_)
  void computeRestLengths()
  {
    Eigen::Matrix<float, 4, 4> T0 =
      buildTransform(0.0f, 0.0f, height_, 0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 6; ++i) {
      Eigen::Matrix<float, 4, 1> d =
        T0 * platform_points_.row(i).transpose() - base_points_.row(i).transpose();

      float L0 = std::sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2));
      rest_lengths_[i] = L0;

      RCLCPP_INFO(
        this->get_logger(),
        "[IK] Rest length leg %d = %.4f", i + 1, L0);
    }
  }

  void handlePlatformPose(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const float x     = static_cast<float>(msg->linear.x);
    const float y     = static_cast<float>(msg->linear.y);
    const float z_cmd = static_cast<float>(msg->linear.z);  // heave around centre
    const float roll  = static_cast<float>(msg->angular.x);
    const float pitch = static_cast<float>(msg->angular.y);
    const float yaw   = static_cast<float>(msg->angular.z);

    // World z = home height + heave command
    const float z_world = height_ + z_cmd;

    RCLCPP_DEBUG(
      this->get_logger(),
      "[IK] Pose cmd -> x=%.3f, y=%.3f, z_cmd=%.3f (z_world=%.3f), roll=%.3f, pitch=%.3f, yaw=%.3f",
      x, y, z_cmd, z_world, roll, pitch, yaw);

    const Eigen::Matrix<float, 4, 4> transform =
      buildTransform(x, y, z_world, roll, pitch, yaw);

    for (int i = 0; i < 6; ++i) {
      Eigen::Matrix<float, 4, 1> d =
        transform * platform_points_.row(i).transpose() - base_points_.row(i).transpose();

      float L      = std::sqrt(d(0)*d(0) + d(1)*d(1) + d(2)*d(2));
      float offset = L - rest_lengths_[i];   // positive = extension, negative = contraction

      legs_msg_.data[i] = offset;

      RCLCPP_DEBUG(
        this->get_logger(),
        "[IK] Leg %d: L=%.4f, L0=%.4f, offset=%.4f",
        i + 1, L, rest_lengths_[i], offset);
    }

    legs_pub_->publish(legs_msg_);
  }

  // Standard roll-pitch-yaw homogeneous transform
  Eigen::Matrix<float, 4, 4> buildTransform(
    float x, float y, float z, float roll, float pitch, float yaw) const
  {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    Eigen::Matrix<float, 4, 4> T;

    T << cy * cp,
        -sy * cr + cy * sp * sr,
         sy * sr + cy * sp * cr,
         x,
         sy * cp,
         cy * cr + sy * sp * sr,
        -cy * sr + sy * sp * cr,
         y,
        -sp,
         cp * sr,
         cp * cr,
         z,
         0.0f, 0.0f, 0.0f, 1.0f;

    return T;
  }

  float height_{0.0f};  // world z at mid-stroke
  Eigen::Matrix<float, 6, 4> base_points_;
  Eigen::Matrix<float, 6, 4> platform_points_;
  std::array<float, 6> rest_lengths_{};

  std_msgs::msg::Float32MultiArray legs_msg_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr legs_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pose_sub_;
};

}  // namespace stewart_platform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stewart_platform::InverseKinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
