#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

class high_level_ctrl : public rclcpp::Node
{
public:
    high_level_ctrl() : Node("high_level_ctrl"), t(0.0)
    {
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&high_level_ctrl::state_callback, this, _1));

        req_puber = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(dt * 1000)),
            std::bind(&high_level_ctrl::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Time-based rotation node initialized.");
    }

private:
    void timer_callback()
    {
        t += dt;

        if (sequence_done) return;

        // =========================
        // 1. STAND UP
        // =========================
        if (!stand_done)
        {
            sport_req.StandUp(req);
            req_puber->publish(req);

            stand_timer += dt;
            if (stand_timer >= 3.0)
            {
                stand_done = true;
                stand_timer = 0.0;
                RCLCPP_INFO(this->get_logger(), "Stand up complete.");
            }
            return;
        }

        // =========================
        // 2. BALANCE
        // =========================
        if (!balance_done)
        {
            sport_req.BalanceStand(req);
            req_puber->publish(req);

            balance_timer += dt;
            if (balance_timer >= 1.0)
            {
                balance_done = true;
                balance_timer = 0.0;
                RCLCPP_INFO(this->get_logger(), "Balance ready.");
            }
            return;
        }

        // =========================
        // 3. MOTION (TIME-BASED)
        // =========================
        if (moving)
        {
            move_timer += dt;

            float vx = 0.7;
            float vy = 0.0;
            float vyaw = 0.7;   // use stable value

            sport_req.Move(req, vx, vy, vyaw);
            req_puber->publish(req);

            // ---- STOP BASED ON TIME ----
            if (move_timer >= max_time)
            {
                sport_req.StopMove(req);
                req_puber->publish(req);

                RCLCPP_INFO(this->get_logger(), "Time-based rotation complete.");

                moving = false;
                sequence_done = true;
            }

            return;
        }

        // =========================
        // 4. START MOVEMENT
        // =========================
        if (!moving && balance_done)
        {
            moving = true;
            move_timer = 0.0;

            RCLCPP_INFO(this->get_logger(), "Starting time-based rotation...");
        }
    }

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // optional logging
        double yaw = data->imu_state.rpy[2];
    }

private:
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_api::msg::Request req;
    SportClient sport_req;

    // timing
    double t;
    double dt = 0.002;

    // state
    bool stand_done = false;
    bool balance_done = false;
    bool moving = false;
    bool sequence_done = false;

    // timers
    double stand_timer = 0.0;
    double balance_timer = 0.0;
    double move_timer = 0.0;

    // duration
    double max_time = 300;  // <-- YOU CONTROL ROTATION HERE
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<high_level_ctrl>());
    rclcpp::shutdown();
    return 0;
}