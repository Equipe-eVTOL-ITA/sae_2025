#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/position.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <cmath>

class PosToRviz : public rclcpp::Node
{
public:
  PosToRviz() : Node("pos_to_rviz")
  {
    rclcpp::QoS qos_pub(1);
    qos_pub.transient_local();
    qos_pub.reliable();
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);
    path_pub_  = create_publisher<nav_msgs::msg::Path>("/drone/path", qos_pub);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/drone/twist", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    sub_ = create_subscription<custom_msgs::msg::Position>(
      "/position", qos,
      [this](const custom_msgs::msg::Position &in)
      {
        if (!std::isfinite(in.x_frd)   || !std::isfinite(in.y_frd)   ||
            !std::isfinite(in.z_frd)   || !std::isfinite(in.yaw_frd) ||
            !std::isfinite(in.vx_frd)  || !std::isfinite(in.vy_frd)  ||
            !std::isfinite(in.vz_frd)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                "Ignoring Position with NaN/inf");
            return;
        }

        rclcpp::Time t = now();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = t;
        pose.header.frame_id = "map";

        pose.pose.position.x = in.x_frd;
        pose.pose.position.y = in.y_frd;
        pose.pose.position.z = -in.z_frd;

        double half = in.yaw_frd * 0.5;
        pose.pose.orientation.w = std::cos(half);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(half);

        pose_pub_->publish(pose);

        path_.header = pose.header;
        path_.poses.push_back(pose);
        path_pub_->publish(path_);

        geometry_msgs::msg::TwistStamped twist;
        twist.header = pose.header;
        twist.twist.linear.x = in.vx_frd;
        twist.twist.linear.y = in.vy_frd;
        twist.twist.linear.z = -in.vz_frd;
        twist_pub_->publish(twist);
      });
  }

private:
  rclcpp::Subscription<custom_msgs::msg::Position>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr            path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosToRviz>());
  rclcpp::shutdown();
  return 0;
}
