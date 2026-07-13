#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class TestStandUpNode : public rclcpp::Node
{
public:
    TestStandUpNode() : Node("test_stand_up_node"), done_(false)
    {
        req_puber_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&TestStandUpNode::run_sequence, this));

        RCLCPP_INFO(this->get_logger(), "TestStandUpNode initialized.");
    }

private:
    void run_sequence()
    {
        if (done_) return;

        timer_->cancel();  // run only once

        RCLCPP_INFO(this->get_logger(), "Standing up...");

        sport_client_.StandUp(req_);
        req_puber_->publish(req_);

        rclcpp::sleep_for(3s);

        RCLCPP_INFO(this->get_logger(), "Stand up complete.");
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

    rclcpp::spin(std::make_shared<TestStandUpNode>());

    rclcpp::shutdown();
    return 0;
}