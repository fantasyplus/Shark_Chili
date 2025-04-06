#ifndef TARKBOT_DRIVER_H
#define TARKBOT_DRIVER_H

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <termios.h>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define PI 3.1415926

// 机器人通信帧头定义
#define ID_CPR2ROS_DATA 0x10 // 下位计向ROS发送的综合数据
#define ID_ROS2CTR_VEL 0x50  // ROS向下位机发送的速度数据
#define ID_ROS2CTR_IMU 0x51  // ROS向下位机发送的IMU陀螺仪零偏校准指令
#define ID_ROS2CTR_RTY 0x5a  // ROS向下位机发送的车型参数
#define ID_ROS2CTR_LGT 0x52  // ROS向下位机发送的灯光调试数据
#define ID_ROS2CTR_LST 0x53  // ROS向下位机发送的灯光保存数据
#define ID_ROS2CTR_BEEP 0x54 // ROS向下位机发送的蜂鸣器数据

#define LIGHT_M1 0x10 // 单色模式
#define LIGHT_M2 0x20 // 呼吸模式

// IMU加速度计量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACC_RATIO (2 * 9.8 / 32768)

// IMU陀螺仪量程±500°，对应数据范围±32768
// 陀螺仪原始数据转换位弧度(rad)单位
#define GYRO_RATIO ((500 * PI / 180) / 32768)

// 机器人数据处理周期,单位S
#define DATA_PERIOD 0.02f

// 数据结构定义
struct imu_data
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

struct imu_orientation_data
{
    float w;
    float x;
    float y;
    float z;
};

struct velocity_data
{
    float linear_x;
    float linear_y;
    float angular_z;
};

struct pose_data
{
    float pos_x;
    float pos_y;
    float angular_z;
};

inline float invSqrt(float x)
{
    volatile long i;
    volatile float halfx, y;
    volatile const float f = 1.5F;

    halfx = x * 0.5F;
    y = x;
    i = *((long *)&y);

    i = 0x5f375a86 - (i >> 1);
    y = *((float *)&i);
    y = y * (f - (halfx * y * y));

    return y;
}

class TarkbotDriver
{
public:
    TarkbotDriver(const std::string &robot_type, const std::string &port, int baud);
    ~TarkbotDriver();

    bool open_serial_port();
    void close_serial_port();

    // 数据访问接口
    imu_data get_imu() const { return imu_data_; }
    imu_orientation_data get_orientation() const { return orient_data_; }
    velocity_data get_velocity() const { return vel_data_; }
    pose_data get_pose() const { return pos_data_; }
    float get_battery() const { return bat_vol_data_; }

    void send_packet(const uint8_t *data, uint8_t len, uint8_t num);

private:
    void async_read();
    void handle_packet(const uint8_t *data);
    void calculateImuQuaternion(imu_data imu_cel);

    boost::asio::io_service io_service_;
    boost::shared_ptr<boost::asio::serial_port> serial_ptr_;
    boost::system::error_code err_code_;

    std::string robot_type_send_;
    std::string port_name_;
    int32_t serial_port_baud_;

    // 数据存储
    imu_data imu_data_;
    imu_orientation_data orient_data_;
    velocity_data vel_data_;
    pose_data pos_data_;
    float bat_vol_data_;

    /***************四元数计算**************************************************/
    volatile float twoKp = 1.0f; // 2 * proportional gain (Kp)
    volatile float twoKi = 0.0f; // 2 * integral gain (Ki)
                                 // quaternion of sensor frame relative to auxiliary frame
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    // integral error terms scaled by Ki
    volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
    volatile const float sampling_period = DATA_PERIOD;
};

#endif // TARKBOT_DRIVER_H