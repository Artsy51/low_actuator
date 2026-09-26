#pragma once

#include <string>
#include <vector>
#include <array>
#include <unordered_map>

// 电机状态结构体
struct MotorState
{
    float position;     // 位置 (rad)
    float velocity;     // 速度 (rad/s)
    float electric;     // 电流 (A)
    float temperature;  // 温度 (°C)
};

// 电机指令结构体
struct MotorCmd
{
    float position;   // 位置指令 (rad)
    float velocity;   // 速度指令 (rad/s)
    float torque;     // 力矩指令 (Nm)
    float kp;         // 位置增益
    float kd;         // 速度增益
};

// 电机参数结构体 - 定义各电机型号的物理参数范围
struct MotorParams
{
    float p_min, p_max;       // 位置范围 (rad)
    float v_min, v_max;       // 速度范围 (rad/s)
    float kp_min, kp_max;     // Kp 增益范围
    float kd_min, kd_max;     // Kd 增益范围
    float t_min, t_max;       // 力矩范围 (Nm)
    float i_min, i_max;       // 电流范围 (A)
};

// 单个电机配置结构体
struct MotorConfig
{
    std::string module_name;         // USB2CAN 模块名称
    int channel;                     // CAN 通道号
    int can_id;                      // CAN ID
    std::string motor_name;          // 电机型号名称
    int axis;                        // 轴向 (1 或 -1)
    float offset;                    // 位置偏移量 (rad)
    std::array<float, 2> pos_limit;  // 位置限制 [min, max] (rad)
};

// 控制配置结构体
struct ControlConfig
{
    int control_frequency;                // 控制频率 (Hz)
    std::string command_topic;            // 指令话题名称
    std::string state_topic;              // 状态话题名称
    std::vector<std::string> joints;      // 关节名称列表
    std::vector<int> joint_mapping;       // 关节映射索引
    int frequency_info;                   // 频率信息标志
    int joints_state_info;                // 关节状态信息标志
};

// CAN 设备管理结构体 - 使用设备名称映射到文件描述符
struct CanDevice
{
    std::unordered_map<std::string, int> devices;

    // 通过设备名称获取文件描述符
    int get_device(const std::string &dev_name) const
    {
        auto it = devices.find(dev_name);
        return (it != devices.end()) ? it->second : -1;
    }

    // 设置设备名称和文件描述符的映射
    void set_device(const std::string &dev_name, int fd)
    {
        devices[dev_name] = fd;
    }

    // 检查设备是否已打开
    bool is_valid(const std::string &dev_name) const
    {
        auto it = devices.find(dev_name);
        return (it != devices.end() && it->second != -1);
    }

    // 关闭所有设备（可选，用于清理）
    void clear()
    {
        devices.clear();
    }

    // 获取已打开的设备数量
    size_t size() const
    {
        return devices.size();
    }
};

