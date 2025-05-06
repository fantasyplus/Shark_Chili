#include "tarkbot_ros_node.hpp"

TarkbotRosNode::TarkbotRosNode()
    : Node("tarkbot_ros_node")
{
    // 初始化参数
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 230400);
    this->declare_parameter<std::string>("robot_type", "r20_akm");
    this->declare_parameter<bool>("pub_odom_tf", false);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    this->declare_parameter<std::string>("imu_frame", "imu_link");

    this->declare_parameter("imu_calibrate", false);
    this->declare_parameter("light_calibrate", false);
    this->declare_parameter("RGB_M", 0);
    this->declare_parameter("RGB_S", 0);
    this->declare_parameter("RGB_T", 0);
    this->declare_parameter("RGB_R", 0);
    this->declare_parameter("RGB_G", 0);
    this->declare_parameter("RGB_B", 0);

    // 注册参数回调函数
    params_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&TarkbotRosNode::handle_parameters, this, std::placeholders::_1));

    this->get_parameter("port", port_);
    this->get_parameter("baud", baud_);
    this->get_parameter("robot_type", robot_type_);
    this->get_parameter("pub_odom_tf", publish_tf_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_footprint_frame", base_footprint_frame_);
    this->get_parameter("imu_frame", imu_frame_);

    // 初始化驱动
    init_driver();
    setup_publishers();
    setup_subscribers();
    setup_services();
    setup_loopback();
}

TarkbotRosNode::~TarkbotRosNode()
{
    driver_.reset();

    RCLCPP_INFO(this->get_logger(), "Tarkbot driver disconnected.");
}

void TarkbotRosNode::init_driver()
{
    try
    {
        // 创建驱动实例
        driver_ = std::make_unique<TarkbotDriver>(robot_type_, port_, baud_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize driver: %s", e.what());
        throw;
    }
}

void TarkbotRosNode::setup_publishers()
{
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_chassis", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void TarkbotRosNode::setup_subscribers()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&TarkbotRosNode::cmd_vel_callback, this, std::placeholders::_1));
    beep_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "beep", 10, std::bind(&TarkbotRosNode::beep_callback, this, std::placeholders::_1));
}

void TarkbotRosNode::setup_services()
{
    light_service_ = this->create_service<tarkbot_driver::srv::LightSet>(
        "light_set", std::bind(&TarkbotRosNode::light_service_callback, this, std::placeholders::_1, std::placeholders::_2));
}

rcl_interfaces::msg::SetParametersResult TarkbotRosNode::handle_parameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    uint8_t data[1];

    rcl_interfaces::msg::SetParametersResult result;
    for (const auto &param : parameters)
    {
        // 处理参数变化逻辑
        if (param.get_name() == "imu_calibrate" && param.as_bool())
        {
            RCLCPP_INFO(get_logger(), "Calibrating the IMU, Please hold the robot stationary for 5 seconds.");
            
            data[0] = 0x55;
            driver_->send_packet(data, 2, ID_ROS2CTR_IMU);
        }
        
        if (param.get_name() == "light_calibrate" && param.as_bool())
        {
            RCLCPP_INFO(get_logger(), "Calibrating the light.");
            
            data[0] = 0x55;
            driver_->send_packet(data, 2, ID_ROS2CTR_LST);
        }
    }
    
    result.successful = true;
    return result;
}

void TarkbotRosNode::setup_loopback()
{
    std::thread loopback_thread(
        [this]()
        {
            while (rclcpp::ok())
            {
                driver_->async_read();

                publish_odom();
                publish_imu();
                publish_battery();
                publish_odom_tf();
            }
        });

    loopback_thread.detach();
}

void TarkbotRosNode::publish_odom()
{
    pose_data pos = driver_->get_pose();
    velocity_data vel = driver_->get_velocity();

    tf2::Quaternion q;
    q.setRPY(0, 0, pos.angular_z);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_footprint_frame_;
    odom_msg.pose.pose.position.x = pos.pos_x;
    odom_msg.pose.pose.position.y = pos.pos_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = vel.linear_x;
    odom_msg.twist.twist.linear.y = vel.linear_y;
    odom_msg.twist.twist.angular.z = vel.angular_z;

    // //里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
    if (vel.linear_x == 0 && vel.linear_y == 0 && vel.angular_z == 0)
    {
        odom_msg.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9};
        odom_msg.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                     0, 1e-3, 1e-9, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e-9};
    }
    else
    {
        odom_msg.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};

        odom_msg.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                     0, 1e-3, 0, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e3};
    }

    // 发布里程计消息
    odom_pub_->publish(odom_msg);
}
void TarkbotRosNode::publish_imu()
{
    // 获取IMU数据
    imu_data imu_data = driver_->get_imu();
    imu_orientation_data orient_data = driver_->get_orientation();

    // 发布IMU数据
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = imu_frame_;
    imu_msg.linear_acceleration.x = imu_data.acc_x;
    imu_msg.linear_acceleration.y = imu_data.acc_y;
    imu_msg.linear_acceleration.z = imu_data.acc_z;
    imu_msg.angular_velocity.x = imu_data.gyro_x;
    imu_msg.angular_velocity.y = imu_data.gyro_y;
    imu_msg.angular_velocity.z = imu_data.gyro_z;
    imu_msg.orientation.w = orient_data.w;
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = orient_data.z;

    // 协方差矩阵
    imu_msg.orientation_covariance[0] = 1e6;
    imu_msg.orientation_covariance[4] = 1e6;
    imu_msg.orientation_covariance[8] = 1e-6;
    imu_msg.angular_velocity_covariance[0] = 1e6;
    imu_msg.angular_velocity_covariance[4] = 1e6;
    imu_msg.angular_velocity_covariance[8] = 1e-6;

    // 发布消息
    imu_pub_->publish(imu_msg);
}

void TarkbotRosNode::publish_battery()
{
    // 获取电池数据
    float battery_data = driver_->get_battery();

    // 发布电池数据
    std_msgs::msg::Float32 battery_msg;
    battery_msg.data = battery_data;

    // 发布消息
    battery_pub_->publish(battery_msg);
}

void TarkbotRosNode::publish_odom_tf()
{
    if (!publish_tf_)
    {
        return;
    }

    tf2::Quaternion q;
    pose_data pos = driver_->get_pose();
    q.setRPY(0, 0, pos.angular_z);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = this->now();
    odom_tf.header.frame_id = odom_frame_;
    odom_tf.child_frame_id = base_footprint_frame_;
    odom_tf.transform.translation.x = pos.pos_x;
    odom_tf.transform.translation.y = pos.pos_y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(odom_tf);
    // RCLCPP_INFO(this->get_logger(), "TF broadcasted: [%s -> %s]", odom_frame_.c_str(), base_footprint_frame_.c_str());
}

void TarkbotRosNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    static uint8_t vel_data[11];

    // 数据转换
    vel_data[0] = static_cast<int16_t>(msg->linear.x * 1000) >> 8;
    vel_data[1] = static_cast<int16_t>(msg->linear.x * 1000);
    vel_data[2] = static_cast<int16_t>(msg->linear.y * 1000) >> 8;
    vel_data[3] = static_cast<int16_t>(msg->linear.y * 1000);
    vel_data[4] = static_cast<int16_t>(msg->angular.z * 1000) >> 8;
    vel_data[5] = static_cast<int16_t>(msg->angular.z * 1000);

    // 设置速度
    driver_->send_packet(vel_data, 6, ID_ROS2CTR_VEL);
    // RCLCPP_INFO(this->get_logger(), "Velocity command sent: [%d, %d, %d, %d, %d, %d]",
    //             vel_data[0], vel_data[1], vel_data[2], vel_data[3], vel_data[4], vel_data[5]);
}

void TarkbotRosNode::beep_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    static uint8_t beep_data[1];

    // 数据转换
    beep_data[0] = static_cast<uint8_t>(msg->data);

    // 设置蜂鸣器
    driver_->send_packet(beep_data, 1, ID_ROS2CTR_BEEP);
    RCLCPP_INFO(this->get_logger(), "Beep command sent: %d", beep_data[0]);
}

void TarkbotRosNode::light_service_callback(
    const std::shared_ptr<tarkbot_driver::srv::LightSet::Request> request,
    const std::shared_ptr<tarkbot_driver::srv::LightSet::Response> response)
{
    static uint8_t light_data[6];
    // 数据转换
    light_data[0] = static_cast<uint8_t>(request->rgb_m);
    light_data[1] = static_cast<uint8_t>(request->rgb_s);
    light_data[2] = static_cast<uint8_t>(request->rgb_t);
    light_data[3] = static_cast<uint8_t>(request->rgb_r);
    light_data[4] = static_cast<uint8_t>(request->rgb_g);
    light_data[5] = static_cast<uint8_t>(request->rgb_b);

    if (light_data[0] == 0xFF || light_data[1] == 0xFF || light_data[2] == 0xFF ||
        light_data[3] == 0xFF || light_data[4] == 0xFF || light_data[5] == 0xFF || light_data[0] > 6)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid light data: [%d, %d, %d, %d, %d, %d]",
                     light_data[0], light_data[1], light_data[2], light_data[3], light_data[4], light_data[5]);
        response->result = "Invalid light data";
        return;
    }

    // 设置灯光
    driver_->send_packet(light_data, 6, ID_ROS2CTR_LGT);
    RCLCPP_INFO(this->get_logger(), "Light command sent: [%d, %d, %d, %d, %d, %d]",
                light_data[0], light_data[1], light_data[2], light_data[3], light_data[4], light_data[5]);

    // 如果没有异常，那么将结果赋值给 response
    response->result = "Success change light";
    RCLCPP_INFO(this->get_logger(), "Light service response: %s", response->result.c_str());
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TarkbotRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
