#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class TestStandSitNode : public rclcpp::Node
{
public:
    TestStandSitNode() : Node("test_stand_sit_node"), done_(false)
    {
        // Publisher
        req_puber_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        // One-shot timer (small delay to allow DDS discovery)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&TestStandSitNode::run_sequence, this));

        RCLCPP_INFO(this->get_logger(), "TestStandSitNode initialized.");
    }

private:
    void run_sequence()
    {
        if (done_) return;

        // Cancel timer immediately so it runs only once
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Standing up...");
        sport_client_.Damp(req_);
        req_puber_->publish(req_);
        done_ = true;
    }

    // ROS2 components
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Unitree components
    unitree_api::msg::Request req_;
    SportClient sport_client_;

    // State
    bool done_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TestStandSitNode>());

    rclcpp::shutdown();
    return 0;
}