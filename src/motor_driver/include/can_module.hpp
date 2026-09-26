#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "can/usb_can.h"
#include "common.hpp"
#include "param_parser.hpp"
#include "motor_decoder.hpp"
#include "utils.hpp"

// CAN 通信模块 - 负责与电机驱动器进行 CAN 总线通信
class CanModule
{
public:
    explicit CanModule(const MotorConfigParser &motor_parser, const ControlConfigParser &ctrl_parser)
        : motor_configs_(motor_parser.get_motors()),
          joints_(ctrl_parser.get_joints())
    {
        // 为每个关节创建解码器
        for (const auto &joint_name : joints_)
        {
            auto config = motor_parser.get_motor(joint_name);
            if (config)
            {
                auto decoder = MotorDecoderFactory::create(config->motor_name);
                if (decoder)
                {
                    decoders_[joint_name] = std::move(decoder);
                }
            }
        }
    }

    ~CanModule() = default;

    // 初始化 CAN 设备
    void init_can()
    {
        // 收集所有需要使用的 USB2CAN 模块
        std::vector<std::string> module_list;
        for (const auto &[joint_name, config] : motor_configs_)
        {
            if (std::find(module_list.begin(), module_list.end(), config.module_name) == module_list.end())
            {
                module_list.push_back(config.module_name);
            }
        }

        // 打开每个 CAN 设备
        for (const auto &module_name : module_list)
        {
            // 从 USB2CAN0 转换为 /dev/USB2CAN0
            std::string dev_path = "/dev/" + module_name;
            int fd = openUSBCAN(dev_path.c_str());
            can_device_.set_device(module_name, fd);

            if (fd == -1)
            {
                std::cout << module_name << " open error" << std::endl;
            }
            else
            {
                std::cout << module_name << " open success" << std::endl;
            }
        }
    }

    // 发送指令到电机
    bool send_command(const std::string &joint_name, const MotorCmd &cmd)
    {
        // 查找关节配置
        auto config_it = motor_configs_.find(joint_name);
        if (config_it == motor_configs_.end())
        {
            std::cout << "Motor config not found for joint: " << joint_name << std::endl;
            return false;
        }
        const auto &config = config_it->second;

        // 查找解码器
        auto decoder_it = decoders_.find(joint_name);
        if (decoder_it == decoders_.end())
        {
            std::cout << "Decoder not found for joint: " << joint_name << std::endl;
            return false;
        }
        auto &decoder = decoder_it->second;

        // 检查设备是否打开
        if (!can_device_.is_valid(config.module_name))
        {
            std::cout << "CAN device not open: " << config.module_name << std::endl;
            return false;
        }

        // 应用轴向变换
        MotorCmd transformed_cmd = cmd;
        apply_axis_transform(transformed_cmd, config, true);

        // 编码指令
        uint8_t data[8];
        if (!decoder->encode(transformed_cmd, data))
        {
            std::cout << "Failed to encode command for joint: " << joint_name << std::endl;
            return false;
        }

        // 构造 CAN 帧
        FrameInfo tx_msg = {
            static_cast<uint32_t>(config.can_id),
            STANDARD,
            8
        };

        // 发送 CAN 帧
        int fd = can_device_.get_device(config.module_name);
        sendUSBCAN(fd, config.channel, &tx_msg, data);
        delay_us(140);

        return true;
    }

    // 接收单帧电机状态
    bool receive_state(const std::string &joint_name, MotorState &state, int timeout_us = 1000)
    {
        // 查找关节配置
        auto config_it = motor_configs_.find(joint_name);
        if (config_it == motor_configs_.end())
        {
            return false;
        }
        const auto &config = config_it->second;

        // 查找解码器
        auto decoder_it = decoders_.find(joint_name);
        if (decoder_it == decoders_.end())
        {
            return false;
        }
        auto &decoder = decoder_it->second;

        // 检查设备是否打开
        if (!can_device_.is_valid(config.module_name))
        {
            return false;
        }

        // 读取 CAN 帧
        FrameInfo info_rx;
        uint8_t channel;
        uint8_t data_rx[8];

        int fd = can_device_.get_device(config.module_name);
        int ret = readUSBCAN(fd, &channel, &info_rx, data_rx, timeout_us);

        if (ret == -1)
        {
            return false;
        }

        // 检查通道和 ID 是否匹配
        if (channel != config.channel || info_rx.canID != static_cast<uint32_t>(config.can_id))
        {
            return false;
        }

        // 检查是否为有效响应帧
        if (!decoder->is_valid_response(data_rx))
        {
            return false;
        }

        // 解码数据
        if (!decoder->decode(data_rx, state))
        {
            return false;
        }

        // 应用轴向变换
        apply_axis_transform(state, config, false);

        return true;
    }

    // Check feedback and command values against 90% of the motor limits.
    bool exceeds_safety_limits(const std::string &joint_name,
                               const MotorState &state,
                               const MotorCmd &command) const
    {
        const auto config_it = motor_configs_.find(joint_name);
        const auto decoder_it = decoders_.find(joint_name);
        if (config_it == motor_configs_.end() || decoder_it == decoders_.end())
        {
            return false;
        }

        const auto &config = config_it->second;
        const auto &params = decoder_it->second->get_params();
        const auto over_limit = [](float value, float min, float max)
        {
            const float limit = std::max(std::abs(min), std::abs(max));
            return limit > 0.0f && std::abs(value) > 0.9f * limit;
        };

        const float position_limit = std::max(std::abs(config.pos_limit[0]),
                                              std::abs(config.pos_limit[1]));
        return (position_limit > 0.0f && std::abs(state.position) > 0.9f * position_limit) ||
               over_limit(state.velocity, params.v_min, params.v_max) ||
               over_limit(state.electric, params.i_min, params.i_max) ||
               over_limit(command.torque, params.t_min, params.t_max) ||
               state.temperature > 80.0f;
    }

private:
    // 应用轴向变换（发送时: motor = (joint - offset) / axis）
    void apply_axis_transform(MotorCmd &cmd, const MotorConfig &config, bool forward)
    {
        if (forward)
        {
            // 发送时: motor = (cmd - offset) / axis
            cmd.position = (cmd.position - config.offset) / config.axis;
            cmd.velocity = cmd.velocity / config.axis;
            cmd.torque = cmd.torque / config.axis;
        }
    }

    // 应用轴向变换（接收时: joint = motor * axis + offset）
    void apply_axis_transform(MotorState &state, const MotorConfig &config, bool forward)
    {
        if (!forward)
        {
            // 接收时: joint = motor * axis + offset
            state.position = state.position * config.axis + config.offset;
            state.velocity = state.velocity * config.axis;
        }
    }

private:
    CanDevice can_device_;
    std::unordered_map<std::string, MotorConfig> motor_configs_;
    std::vector<std::string> joints_;
    std::unordered_map<std::string, std::unique_ptr<MotorDecoder>> decoders_;
};
