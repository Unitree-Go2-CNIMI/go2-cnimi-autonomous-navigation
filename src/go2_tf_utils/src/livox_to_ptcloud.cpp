#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iomanip>

using std::placeholders::_1;

class LivoxToPointCloud2Node : public rclcpp::Node
{
public:
    LivoxToPointCloud2Node()
    : Node("livox2pointcloud_node"),
      lidar_mean_scantime_(0.1),
      scan_num_(0)
    {
        this->declare_parameter<std::string>("pointcloud_topic", "/livox/pointcloud2");
        this->declare_parameter<std::string>("livox_topic", "/livox/lidar");

        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        livox_topic_ = this->get_parameter("livox_topic").as_string();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);

        sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, 10,
            std::bind(&LivoxToPointCloud2Node::callback, this, _1)
        );

        // RCLCPP_INFO(this->get_logger(), "Listening on: %s", livox_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Publishing to: %s", pointcloud_topic_.c_str());
    }

private:

    sensor_msgs::msg::PointCloud2 convert(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;

        size_t n = livox_msg->points.size();
        if (n == 0)
            return cloud_msg;

        rclcpp::Time start_time = livox_msg->header.stamp;

        double last_point_time =
            livox_msg->points.back().offset_time / float(1000000);

        double lidar_end_time_offset = 0.0;

        if (n <= 1)
        {
            lidar_end_time_offset = lidar_mean_scantime_;
        }
        else if (last_point_time / double(1000)
                < 0.5 * lidar_mean_scantime_)
        {
            lidar_end_time_offset = lidar_mean_scantime_;
        }
        else
        {
            scan_num_++;

            lidar_end_time_offset =
                last_point_time / double(1000);

            lidar_mean_scantime_ +=
                (last_point_time / double(1000)
                - lidar_mean_scantime_)
                / scan_num_;
        }

        rclcpp::Time end_time =
            start_time +
            rclcpp::Duration::from_seconds(
                lidar_end_time_offset);

        cloud_msg.header = livox_msg->header;
        //cloud_msg.header.stamp = end_time;
        cloud_msg.height = 1;
        cloud_msg.width = n;

        cloud_msg.fields.resize(7);
        cloud_msg.fields[0] = createField("time", 0, sensor_msgs::msg::PointField::FLOAT32);
        cloud_msg.fields[1] = createField("x", 4, sensor_msgs::msg::PointField::FLOAT32);
        cloud_msg.fields[2] = createField("y", 8, sensor_msgs::msg::PointField::FLOAT32);
        cloud_msg.fields[3] = createField("z", 12, sensor_msgs::msg::PointField::FLOAT32);
        cloud_msg.fields[4] = createField("intensity", 16, sensor_msgs::msg::PointField::FLOAT32);
        cloud_msg.fields[5] = createField("tag", 20, sensor_msgs::msg::PointField::UINT8);
        cloud_msg.fields[6] = createField("line", 21, sensor_msgs::msg::PointField::UINT8);

        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;

        cloud_msg.point_step = 22;
        cloud_msg.row_step = cloud_msg.point_step * n;
        cloud_msg.data.resize(cloud_msg.row_step);

        sensor_msgs::PointCloud2Iterator<float> iter_time(cloud_msg, "time");
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");

        for (size_t i = 0; i < n; ++i)
        {
            const auto &p = livox_msg->points[i];

            *iter_time = p.offset_time / float(1000000);
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            *iter_intensity = p.reflectivity;
            *iter_tag = p.tag;
            *iter_line = p.line;

            ++iter_time;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
            ++iter_tag;
            ++iter_line;
        }

        return cloud_msg;
    }

    sensor_msgs::msg::PointField createField(
        const std::string & name, uint32_t offset, uint8_t datatype)
    {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = datatype;
        field.count = 1;
        return field;
    }
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        auto now_time = this->now();

        auto cloud = convert(msg);

        rclcpp::Time scan_begin = msg->header.stamp;
        rclcpp::Time scan_end = cloud.header.stamp;

        double scan_duration =
            (scan_end - scan_begin).seconds();

        double delay =
            (now_time - scan_end).seconds();

        // std::cout
        //     << std::fixed << std::setprecision(9)
        //     << "\n========== TIMING ==========\n"
        //     << "Message received now : " << now_time.seconds() << "\n"
        //     << "Scan begin stamp     : " << scan_begin.seconds() << "\n"
        //     << "Scan end stamp       : " << scan_end.seconds() << "\n"
        //     << "Published stamp      : " << rclcpp::Time(cloud.header.stamp).seconds()<< "\n"
        //     << "Scan duration        : " << scan_duration << " s\n"
        //     << "Current delay        : " << delay << " s\n"
        //     << "============================\n";

        pub_->publish(cloud);

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        //     "Publishing Fast-LIO MID360 compatible PointCloud2");
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;

    std::string pointcloud_topic_;
    std::string livox_topic_;

    // Fast-LIO scan time estimation
    double lidar_mean_scantime_;
    int scan_num_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPointCloud2Node>());
    rclcpp::shutdown();
    return 0;
}