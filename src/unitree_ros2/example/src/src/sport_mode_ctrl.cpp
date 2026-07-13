#include <unistd.h>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;
using namespace std::chrono_literals; 

class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&soprt_request::state_callback, this, _1));

        req_puber = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(dt * 1000)),
            std::bind(&soprt_request::timer_callback, this));

        t = -1;
    };

private:
    void timer_callback()
    {
        if (done_) return;

        if (!init_received)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial state...");
            return;
        }

        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Standing up...");
        sport_req.StandUp(req);
        req_puber->publish(req);
        rclcpp::sleep_for(3s);

        RCLCPP_INFO(this->get_logger(), "Balancing...");
        sport_req.BalanceStand(req);
        req_puber->publish(req);
        rclcpp::sleep_for(1s);

        RCLCPP_INFO(this->get_logger(), "Moving forward...");

        double time_temp = 0.0;
        double time_seg = 0.2;
        double v = 0.3;

        std::vector<PathPoint> path;

        for (int i = 0; i < 10; i++)
        {
            PathPoint path_point_tmp;
            time_temp  = i * time_seg;

            float px_local = v * time_temp;
            float py_local = 0;
            float yaw_local = 0.;
            float vx_local;
            if (i == 0)
                vx_local = 0.0;
            else
                vx_local = v;
            float vy_local = 0;
            float vyaw_local = 0.;

            path_point_tmp.timeFromStart = i * time_seg;
            path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
            path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
            path_point_tmp.yaw = yaw_local + yaw0;
            path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
            path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
            path_point_tmp.vyaw = vyaw_local;

            path.push_back(path_point_tmp);
        }

        sport_req.TrajectoryFollow(req, path);
        req_puber->publish(req);

        done_ = true;
        RCLCPP_INFO(this->get_logger(), "Sequence complete.");
    };

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (!init_received)
        {
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            init_received = true;

            std::cout << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req;
    SportClient sport_req;

    double t;
    double dt = 0.002;

    bool init_received = false;
    bool done_ = false; 

    double px0 = 0;
    double py0 = 0;
    double yaw0 = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<soprt_request>());
    rclcpp::shutdown();
    return 0;
}