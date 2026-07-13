#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class SitDownNode : public rclcpp::Node
{
public:
    SitDownNode() : Node("sit_down_node"), done_(false)
    {
        req_puber_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&SitDownNode::run, this));

        RCLCPP_INFO(this->get_logger(), "SitDownNode initialized.");
    }

private:
    void run()
    {
        if (done_) return;

        timer_->cancel();  // run once

        RCLCPP_INFO(this->get_logger(), "Sending SitDown command...");

        sport_client_.StandDown(req_);
        req_puber_->publish(req_);

        RCLCPP_INFO(this->get_logger(), "SitDown command sent.");

        done_ = true;
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_api::msg::Request req_;
    SportClient sport_client_;

    bool done_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SitDownNode>());
    rclcpp::shutdown();
    return 0;
}