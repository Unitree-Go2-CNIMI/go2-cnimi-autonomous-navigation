#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

class FastLioAlignedTF : public rclcpp::Node
{
public:
  FastLioAlignedTF() : Node("fastlio_aligned_tf")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_enc_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", 10,
      std::bind(&FastLioAlignedTF::odomCallback, this, std::placeholders::_1));

    sub_lio_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&FastLioAlignedTF::lioCallback, this, std::placeholders::_1));

    // base_link → livox_frame
    T_base_livox_ = makeTransform(0.13, -0.02, 0.15, Eigen::Quaterniond(1,0,0,0));
    T_livox_base_ = T_base_livox_.inverse();
  }

private:

  // ---------- Callbacks ----------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    enc_odom_ = msg;
  }

  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Matrix4d T_lio = odomToMatrix(msg);

    if (!T_align_initialized_)
    {
      if (!enc_odom_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for encoder odom...");
        return;
      }

      Eigen::Matrix4d T_enc = odomToMatrix(enc_odom_);
      Eigen::Matrix4d T_enc_livox = T_enc * T_base_livox_;

      T_align_ = T_enc_livox * T_lio.inverse();
      T_align_initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "Alignment initialized");
    }

    Eigen::Matrix4d T_odom_livox = T_align_ * T_lio;
    Eigen::Matrix4d T_odom_base = T_odom_livox * T_livox_base_;

    publishTF(T_odom_base, msg->header.stamp);
  }

  // ---------- Math helpers ----------

  Eigen::Matrix4d makeTransform(double x, double y, double z, const Eigen::Quaterniond &q)
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    return T;
  }

  Eigen::Matrix4d odomToMatrix(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;

    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = quat.toRotationMatrix();
    T(0,3) = p.x;
    T(1,3) = p.y;
    T(2,3) = p.z;

    return T;
  }

  void publishTF(const Eigen::Matrix4d &T, const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    Eigen::Vector3d trans = T.block<3,1>(0,3);
    Eigen::Matrix3d rot = T.block<3,3>(0,0);
    Eigen::Quaterniond q(rot);

    t.transform.translation.x = trans.x();
    t.transform.translation.y = trans.y();
    t.transform.translation.z = trans.z();

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  // ---------- Members ----------

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_enc_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_;

  nav_msgs::msg::Odometry::SharedPtr enc_odom_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Eigen::Matrix4d T_base_livox_;
  Eigen::Matrix4d T_livox_base_;

  Eigen::Matrix4d T_align_;
  bool T_align_initialized_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioAlignedTF>());
  rclcpp::shutdown();
  return 0;
}