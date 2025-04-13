#include "tarkbot_chasis_driver.hpp"

TarkbotDriver::TarkbotDriver(const std::string &robot_type, const std::string &port, int baud)
    : robot_type_send_(robot_type), port_name_(port), serial_port_baud_(baud)
{
    if (port_name_.empty())
    {
        throw std::runtime_error("Serial port is empty");
    }
    if (serial_port_baud_ <= 0)
    {
        throw std::runtime_error("Invalid baud rate");
    }

    if (!open_serial_port())
    {
        throw std::runtime_error("Failed to open serial port");
    }

    // 设置车型
    static uint8_t cartype_data[1];
    if (robot_type_send_ == "r20_mec")
        cartype_data[0] = 1;
    if (robot_type_send_ == "r20_fwd")
        cartype_data[0] = 2;
    if (robot_type_send_ == "r20_akm")
        cartype_data[0] = 3;
    if (robot_type_send_ == "r20_twd")
        cartype_data[0] = 4;
    if (robot_type_send_ == "r20_tak")
        cartype_data[0] = 5;
    if (robot_type_send_ == "r20_omni")
        cartype_data[0] = 6;
    send_packet(cartype_data, 1, ID_ROS2CTR_RTY);
}

TarkbotDriver::~TarkbotDriver()
{
    static uint8_t vel_data[11];
    static uint8_t beep_data[1];

    // 发送静止指令
    vel_data[0] = 0;
    vel_data[1] = 0;
    vel_data[2] = 0;
    vel_data[3] = 0;
    vel_data[4] = 0;
    vel_data[5] = 0;
    send_packet(vel_data, 6, ID_ROS2CTR_VEL);

    // 数据转换
    beep_data[0] = 0;

    // 发送串口数据
    send_packet(beep_data, 1, ID_ROS2CTR_BEEP);

    // 关闭串口
    close_serial_port();
}

bool TarkbotDriver::open_serial_port()
{
    // 检查串口是否已经被打开
    if (serial_ptr_)
    {
        return false;
    }

    // 打开串口
    serial_ptr_ = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io_service_));
    serial_ptr_->open(port_name_, err_code_);

    // 串口是否正常打开
    if (err_code_)
    {
        std::cerr << "Open Port: " << port_name_ << " Failed! Abort!" << std::endl;
        std::cerr << "Error: " << err_code_.message() << std::endl;
        return false;
    }

    // 初始化串口参数
    serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
    serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    return true;
}
void TarkbotDriver::close_serial_port()
{
    // 如果串口被打开，则关闭串口
    if (serial_ptr_)
    {
        serial_ptr_->cancel();
        serial_ptr_->close();
        serial_ptr_.reset();
    }

    // 停止IO服务
    io_service_.stop();
    io_service_.reset();
}

void TarkbotDriver::async_read()
{
    // 协议解析变量
    static uint8_t rx_con = 0;  // 接收计数器
    static uint8_t rx_checksum; // 帧头部分校验和
    static uint8_t rx_buf[60];  // 接收缓冲
    uint8_t res;

    // 读取串口数据
    boost::asio::read(*serial_ptr_.get(), boost::asio::buffer(&res, 1), err_code_);

    // 塔克X-Protocol协议解析数据
    if (rx_con < 3) //=========接收帧头 + 长度
    {
        if (rx_con == 0) // 接收帧头1 0xAA
        {
            if (res == 0xAA)
            {
                rx_buf[0] = res;
                rx_con = 1;
            }
            else
            {
            }
        }
        else if (rx_con == 1) // 接收帧头2 0x55
        {
            if (res == 0x55)
            {
                rx_buf[1] = res;
                rx_con = 2;
            }
            else
            {
                rx_con = 0;
            }
        }
        else // 接收数据长度
        {
            rx_buf[2] = res;
            rx_con = 3;
            rx_checksum = (0xAA + 0x55) + res; // 计算校验和
        }
    }
    else //=========接收数据
    {
        if (rx_con < (rx_buf[2] - 1))
        {
            rx_buf[rx_con] = res;
            rx_con++;
            rx_checksum = rx_checksum + res;
        }
        else // 判断最后1位
        {
            // 接收完成，恢复初始状态
            rx_con = 0;

            // 数据校验
            if (res == rx_checksum) // 校验正确
            {
                handle_packet(rx_buf);
            }
        }
    }
}

void TarkbotDriver::calculateImuQuaternion(imu_data imu_cel)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = invSqrt(imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

    imu_cel.acc_x *= recipNorm;
    imu_cel.acc_y *= recipNorm;
    imu_cel.acc_z *= recipNorm;

    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
    halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
    halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

    // 计算并应用积分反馈（如果启用）
    if (twoKi > 0.0f)
    {
        integralFBx += twoKi * halfex * sampling_period; // integral error scaled by Ki
        integralFBy += twoKi * halfey * sampling_period;
        integralFBz += twoKi * halfez * sampling_period;
        imu_cel.gyro_x += integralFBx; // apply integral feedback
        imu_cel.gyro_y += integralFBy;
        imu_cel.gyro_z += integralFBz;
    }
    else
    {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // Apply proportional feedback
    imu_cel.gyro_x += twoKp * halfex;
    imu_cel.gyro_y += twoKp * halfey;
    imu_cel.gyro_z += twoKp * halfez;

    // Integrate rate of change of quaternion
    imu_cel.gyro_x *= (0.5f * sampling_period); // pre-multiply common factors
    imu_cel.gyro_y *= (0.5f * sampling_period);
    imu_cel.gyro_z *= (0.5f * sampling_period);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - q3 * imu_cel.gyro_z);
    q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - q3 * imu_cel.gyro_y);
    q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + q3 * imu_cel.gyro_x);
    q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 计算结果赋值到
    orient_data_.w = q0;
    orient_data_.x = q1;
    orient_data_.y = q2;
    orient_data_.z = q3;

    // 计算欧拉角
    //  roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    //  pitch = asinf(-2.0f * (q1*q3 - q0*q2));
    //  yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    //  ROS_INFO("IMU:%f  %f  %f ",roll,pitch,yaw);
}

void TarkbotDriver::handle_packet(const uint8_t *data)
{
    if (data[3] == ID_CPR2ROS_DATA)
    {
        // 解析IMU加速度数据
        imu_data_.acc_x = ((double)((int16_t)(data[4] * 256 + data[5])) * ACC_RATIO);
        imu_data_.acc_y = ((double)((int16_t)(data[6] * 256 + data[7])) * ACC_RATIO);
        imu_data_.acc_z = ((double)((int16_t)(data[8] * 256 + data[9])) * ACC_RATIO);

        // 解析IMU陀螺仪数据
        imu_data_.gyro_x = ((double)((int16_t)(data[10] * 256 + data[11])) * GYRO_RATIO);
        imu_data_.gyro_y = ((double)((int16_t)(data[12] * 256 + data[13])) * GYRO_RATIO);
        imu_data_.gyro_z = ((double)((int16_t)(data[14] * 256 + data[15])) * GYRO_RATIO);

        // 计算IMU四元数数据
        calculateImuQuaternion(imu_data_);

        // 解析机器人速度
        vel_data_.linear_x = ((double)((int16_t)(data[16] * 256 + data[17])) / 1000);
        vel_data_.linear_y = ((double)((int16_t)(data[18] * 256 + data[19])) / 1000);
        vel_data_.angular_z = ((double)((int16_t)(data[20] * 256 + data[21])) / 1000);

        // 解析电压值
        bat_vol_data_ = (double)(((data[22] << 8) + data[23])) / 100;

        // 计算里程计数据
        pos_data_.pos_x += (vel_data_.linear_x * cos(pos_data_.angular_z) - vel_data_.linear_y * sin(pos_data_.angular_z)) * DATA_PERIOD;
        pos_data_.pos_y += (vel_data_.linear_x * sin(pos_data_.angular_z) + vel_data_.linear_y * cos(pos_data_.angular_z)) * DATA_PERIOD;
        pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD; // 绕Z轴的角位移，单位：rad
    }
}

void TarkbotDriver::send_packet(const uint8_t *data, uint8_t len, uint8_t num)
{
    uint8_t i, cnt;
    uint8_t tx_checksum = 0; // 发送校验和
    uint8_t tx_buf[64];

    // 判断是否超出长度
    if (len <= 64)
    {
        // 获取数据
        tx_buf[0] = 0xAA;    // 帧头
        tx_buf[1] = 0x55;    //
        tx_buf[2] = len + 5; // 根据输出长度计算帧长度
        tx_buf[3] = num;     // 帧编码

        for (i = 0; i < len; i++)
        {
            tx_buf[4 + i] = *(data + i);
        }

        // 计算校验和
        cnt = 4 + len;
        for (i = 0; i < cnt; i++)
        {
            tx_checksum = tx_checksum + tx_buf[i];
        }
        tx_buf[i] = tx_checksum;

        // 计算帧长度
        cnt = len + 5;

        // 发送数据
        boost::asio::write(*serial_ptr_.get(), boost::asio::buffer(tx_buf, cnt), err_code_);
    }
}