// tarkbot_ros_node.hpp - ROS 2接口部分
#ifndef TARKBOT_ROBOT_H
#define TARKBOT_ROBOT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tarkbot_chasis_driver.hpp"
#include "tarkbot_driver/srv/light_set.hpp"

using namespace std::chrono_literals;

class TarkbotRosNode : public rclcpp::Node
{
public:
    explicit TarkbotRosNode();
    ~TarkbotRosNode();

private:
    void init_driver();
    void setup_publishers();
    void setup_subscribers();
    void setup_services();
    void setup_loopback();

    // 回调函数
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void beep_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void light_service_callback(
        const std::shared_ptr<tarkbot_driver::srv::LightSet::Request> request,
        const std::shared_ptr<tarkbot_driver::srv::LightSet::Response> response);

    // 定时发布函数
    void publish_odom();
    void publish_imu();
    void publish_battery();

    // TF广播
    void publish_odom_tf();

    // 参数更新处理
    rcl_interfaces::msg::SetParametersResult handle_parameters(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    std::unique_ptr<TarkbotDriver> driver_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS接口
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr beep_sub_;
    rclcpp::Service<tarkbot_driver::srv::LightSet>::SharedPtr light_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 参数存储
    std::string port_;
    int baud_;
    bool publish_tf_;
    std::string robot_type_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    std::string imu_frame_;
};

#endif // TARKBOT_ROBOT_H