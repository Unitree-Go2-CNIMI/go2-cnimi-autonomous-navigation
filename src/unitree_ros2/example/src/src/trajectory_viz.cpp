#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom", 10,
            std::bind(&TrajectoryPublisher::odom_callback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/trajectory", 10);

        path_.header.frame_id = "odom";

        RCLCPP_INFO(this->get_logger(), "Trajectory publisher started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;

        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        path_.header.stamp = this->now();
        path_.poses.push_back(pose);

        path_pub_->publish(path_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}