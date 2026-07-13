#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class StabilizedBase : public rclcpp::Node
{
public:
    StabilizedBase()
    : Node("stabilized_base")
    {
        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&StabilizedBase::update, this));
    }

private:
    void update()
    {
        geometry_msgs::msg::TransformStamped tf;

        try
        {
            tf = tf_buffer_->lookupTransform(
                "odom",
                "livox_frame",
                tf2::TimePointZero);
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // Current quaternion
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        // Extract RPY
        double roll, pitch, yaw;

        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Remove roll and pitch
        tf2::Quaternion q_stabilized;
        q_stabilized.setRPY(0.0, 0.0, yaw);
        q_stabilized.normalize();

        geometry_msgs::msg::TransformStamped out;

        // Use original TF timestamp
        out.header.stamp = tf.header.stamp;

        out.header.frame_id = "odom";
        out.child_frame_id = "livox_frame_stabilized";

        // Keep same translation
        out.transform.translation = tf.transform.translation;

        // New stabilized rotation
        out.transform.rotation.x = q_stabilized.x();
        out.transform.rotation.y = q_stabilized.y();
        out.transform.rotation.z = q_stabilized.z();
        out.transform.rotation.w = q_stabilized.w();

        tf_broadcaster_->sendTransform(out);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StabilizedBase>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}