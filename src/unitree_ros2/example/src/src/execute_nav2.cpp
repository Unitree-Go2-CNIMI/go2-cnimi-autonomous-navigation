#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "go2_msgs/srv/start_cmd_vel_bridge.hpp"
#include "go2_msgs/srv/stop_cmd_vel_bridge.hpp"
#include "go2_msgs/srv/get_cmd_vel_bridge_status.hpp"

using StartSrv  = go2_msgs::srv::StartCmdVelBridge;
using StopSrv   = go2_msgs::srv::StopCmdVelBridge;
using StatusSrv = go2_msgs::srv::GetCmdVelBridgeStatus;

static constexpr double CTRL_HZ          = 50.0;
static constexpr double CTRL_DT          = 1.0 / CTRL_HZ;
static constexpr double STAND_DURATION   = 3.0;
static constexpr double BALANCE_DURATION = 1.0;

class CmdVelBridge : public rclcpp::Node
{
public:
    CmdVelBridge() : Node("cmd_vel_bridge")
    {
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_out", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_vx_   = static_cast<float>(msg->linear.x);
                last_vy_   = static_cast<float>(msg->linear.y);
                last_vyaw_ = static_cast<float>(msg->angular.z);
            },
            sub_opts);

        req_pub_    = create_publisher<unitree_api::msg::Request>(
            "/api/sport/request",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());
        status_pub_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel_bridge/status", 10);

        start_srv_ = create_service<StartSrv>(
            "start_cmd_vel_bridge",
            [this](const StartSrv::Request::SharedPtr,
                   StartSrv::Response::SharedPtr res)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (bridge_active_) {
                    res->success = false;
                    res->message = "Bridge already running";
                    return;
                }
                stand_done_    = false;
                startup_done_  = false;
                startup_timer_ = 0.0;
                bridge_active_ = true;
                RCLCPP_INFO(get_logger(), "Bridge started — running startup sequence.");
                res->success = true;
                res->message = "Bridge started";
            },
            rmw_qos_profile_services_default, cb_group_);

        stop_srv_ = create_service<StopSrv>(
            "stop_cmd_vel_bridge",
            [this](const StopSrv::Request::SharedPtr,
                   StopSrv::Response::SharedPtr res)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!bridge_active_) {
                        res->success = false;
                        res->message = "Bridge not running";
                        return;
                    }
                    bridge_active_ = false;
                    stand_done_    = false;
                    startup_done_  = false;
                    startup_timer_ = 0.0;
                } // lock released before publish
                unitree_api::msg::Request req;
                sport_req_.StopMove(req);
                req_pub_->publish(req);
                RCLCPP_INFO(get_logger(), "Bridge stopped.");
                res->success = true;
                res->message = "Bridge stopped";
            },
            rmw_qos_profile_services_default, cb_group_);

        status_srv_ = create_service<StatusSrv>(
            "get_cmd_vel_bridge_status",
            [this](const StatusSrv::Request::SharedPtr,
                   StatusSrv::Response::SharedPtr res)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                res->running = bridge_active_;
                res->message = !bridge_active_ ? "Idle"
                             : !startup_done_  ? "Starting"
                                               : "Running";
            },
            rmw_qos_profile_services_default, cb_group_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(CTRL_DT * 1000.0)),
            [this]() { timerCallback(); },
            cb_group_);

        RCLCPP_INFO(get_logger(), "CmdVel bridge ready — call start_cmd_vel_bridge to begin.");
    }

private:
    void timerCallback()
    {
        // Snapshot shared state under lock, then publish outside it so the
        // lock is never held while calling into the DDS/transport layer.
        bool  active, stand_done, startup_done;
        float vx, vy, vyaw;
        double startup_timer;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            active       = bridge_active_;
            stand_done   = stand_done_;
            startup_done = startup_done_;
            startup_timer = startup_timer_;
            vx   = last_vx_;
            vy   = last_vy_;
            vyaw = last_vyaw_;
        }

        // Always publish status so the frontend can subscribe via rosbridge
        {
            std_msgs::msg::Bool s;
            s.data = active;
            status_pub_->publish(s);
        }

        if (!active) return;

        if (!startup_done) {
            startup_timer += CTRL_DT;

            unitree_api::msg::Request req;

            if (!stand_done) {
                sport_req_.StandUp(req);
                req_pub_->publish(req);

                if (startup_timer >= STAND_DURATION) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stand_done_    = true;
                    startup_timer_ = 0.0;
                    RCLCPP_INFO(get_logger(), "Stand up complete.");
                } else {
                    std::lock_guard<std::mutex> lock(mutex_);
                    startup_timer_ = startup_timer;
                }
                return;
            }

            sport_req_.BalanceStand(req);
            req_pub_->publish(req);

            if (startup_timer >= BALANCE_DURATION) {
                std::lock_guard<std::mutex> lock(mutex_);
                startup_done_  = true;
                RCLCPP_INFO(get_logger(), "Balance stand ready — accepting cmd_vel.");
            } else {
                std::lock_guard<std::mutex> lock(mutex_);
                startup_timer_ = startup_timer;
            }
            return;
        }

        // Normal operation — each command uses a fresh local request
        unitree_api::msg::Request req;
        if (vx == 0.0f && vy == 0.0f && vyaw == 0.0f)
            sport_req_.StopMove(req);
        else
            sport_req_.Move(req, vx, vy, vyaw);
        req_pub_->publish(req);
    }

    rclcpp::CallbackGroup::SharedPtr                           cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr    req_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          status_pub_;
    rclcpp::TimerBase::SharedPtr                               timer_;
    rclcpp::Service<StartSrv>::SharedPtr                       start_srv_;
    rclcpp::Service<StopSrv>::SharedPtr                        stop_srv_;
    rclcpp::Service<StatusSrv>::SharedPtr                      status_srv_;

    SportClient sport_req_;
    std::mutex  mutex_;

    float  last_vx_   = 0.0f;
    float  last_vy_   = 0.0f;
    float  last_vyaw_ = 0.0f;

    bool   bridge_active_  = false;
    bool   stand_done_     = false;
    bool   startup_done_   = false;
    double startup_timer_  = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelBridge>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
