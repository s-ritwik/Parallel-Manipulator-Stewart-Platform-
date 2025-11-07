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
    printf("[IK] Node constructor called\n");

    height_ = static_cast<float>(
      this->declare_parameter("platform_home_height", 2.0625));
    printf("[IK] Parameter declared: platform_home_height = %.4f\n", height_);

    initAttachmentPoints();
    printf("[IK] Attachment points initialized\n");

    initMessage();
    printf("[IK] Message initialized\n");

    legs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_position_cmd", rclcpp::SystemDefaultsQoS());
    printf("[IK] Publisher created: /stewart/legs_position_cmd\n");

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "stewart/platform_pose", rclcpp::SensorDataQoS(),
      std::bind(&InverseKinematicsNode::handlePlatformPose, this, std::placeholders::_1));
    printf("[IK] Subscriber created: stewart/platform_pose\n");

    RCLCPP_INFO(this->get_logger(), "Inverse kinematics node ready");
    printf("[IK] Node ready and spinning\n");
  }

private:
  void initAttachmentPoints()
  {
    base_points_ << 1.5588457268119897, 0.9, 0.25, 1,
      1.1021821192326179e-16, 1.8, 0.25, 1,
      -1.5588457268119897, 0.9, 0.25, 1,
      -1.5588457268119895, -0.9, 0.25, 1,
      -3.3065463576978537e-16, -1.8, 0.25, 1,
      1.558845726811989, -0.9000000000000008, 0.25, 1;

    platform_points_ << 0.9545941546018393, 0.9545941546018392, -0.05, 1,
      0.34940571088840333, 1.303999865490242, -0.05, 1,
      -1.303999865490242, 0.3494057108884034, -0.05, 1,
      -1.3039998654902425, -0.3494057108884025, -0.05, 1,
      0.34940571088840355, -1.303999865490242, -0.05, 1,
      0.9545941546018399, -0.9545941546018385, -0.05, 1;
  }

  void initMessage()
  {
    legs_msg_.data.assign(6, 0.0F);
  }

  void handlePlatformPose(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    printf("[IK] Received platform pose message\n");

    const float x = static_cast<float>(msg->linear.x);
    const float y = static_cast<float>(msg->linear.y);
    const float z = static_cast<float>(msg->linear.z);
    const float roll = static_cast<float>(msg->angular.x);
    const float pitch = static_cast<float>(msg->angular.y);
    const float yaw = static_cast<float>(msg->angular.z);

    printf("[IK] Pose -> x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f\n",
           x, y, z, roll, pitch, yaw);

    const Eigen::Matrix<float, 4, 4> transform =
      buildTransform(x, y, z + height_, roll, pitch, yaw);
    printf("[IK] Transform matrix built\n");

    for (int i = 0; i < 6; ++i) {
      const Eigen::Matrix<float, 4, 1> length =
        transform * platform_points_.row(i).transpose() - base_points_.row(i).transpose();
      legs_msg_.data[i] = std::sqrt(
        length(0) * length(0) + length(1) * length(1) + length(2) * length(2)) - height_;
      printf("[IK] Leg %d length = %.4f\n", i + 1, legs_msg_.data[i]);
    }

    legs_pub_->publish(legs_msg_);
    printf("[IK] Published legs position command\n");
  }

  Eigen::Matrix<float, 4, 4> buildTransform(
    float x, float y, float z, float roll, float pitch, float yaw) const
  {
    Eigen::Matrix<float, 4, 4> transform;
    transform << std::cos(yaw) * std::cos(pitch),
      -std::sin(yaw) * std::cos(roll) + std::cos(yaw) * std::sin(pitch) * std::sin(roll),
      std::sin(yaw) * std::sin(roll) + std::cos(yaw) * std::sin(pitch) * std::cos(roll),
      x,
      std::sin(yaw) * std::cos(pitch),
      std::cos(yaw) * std::cos(roll) + std::sin(yaw) * std::sin(pitch) * std::sin(roll),
      -std::cos(yaw) * std::sin(roll) + std::sin(yaw) * std::sin(pitch) * std::cos(roll),
      y,
      -std::sin(pitch),
      std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll),
      z,
      0, 0, 0, 1;

    printf("[IK] Transform built for pose (%.2f, %.2f, %.2f)\n", x, y, z);
    return transform;
  }

  float height_{};
  Eigen::Matrix<float, 6, 4> base_points_;
  Eigen::Matrix<float, 6, 4> platform_points_;

  std_msgs::msg::Float32MultiArray legs_msg_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr legs_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pose_sub_;
};

}  // namespace stewart_platform


int main(int argc, char ** argv)
{
  printf("[MAIN] Starting Inverse Kinematics Node...\n");
  rclcpp::init(argc, argv);
  printf("[MAIN] rclcpp initialized\n");

  auto node = std::make_shared<stewart_platform::InverseKinematicsNode>();
  printf("[MAIN] Node object created\n");

  rclcpp::spin(node);
  printf("[MAIN] Spinning finished, shutting down\n");

  rclcpp::shutdown();
  printf("[MAIN] rclcpp shutdown complete\n");
  return 0;
}
