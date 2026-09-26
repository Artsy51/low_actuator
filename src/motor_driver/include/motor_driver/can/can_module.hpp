#pragma once

#include <iostream>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "can/usb_can.h"
#include "motor_driver/core/common.hpp"
#include "motor_driver/config/param_parser.hpp"
#include "motor_driver/can/motor_decoder.hpp"
#include "motor_driver/core/utils.hpp"

// CAN 通信模块 - 负责与电机驱动器进行 CAN 总线通信
class CanModule
{
public:
    explicit CanModule(const MotorConfigParser &motor_parser, const ControlConfigParser &)
        : motor_configs_(motor_parser.get_motors())
    {
        for (const auto &[joint_name, config] : motor_configs_)
        {
            auto decoder = MotorDecoderFactory::create(config.motor_name);
            if (decoder)
            {
                decoders_[joint_name] = std::move(decoder);
            }
        }
    }

    ~CanModule()
    {
        for (const auto &[module, fd] : can_device_.devices)
        {
            if (fd >= 0)
            {
                closeUSBCAN(fd);
            }
        }
    }

    // 初始化 CAN 设备
    bool init_can()
    {
        std::map<std::string, bool> modules;
        for (const auto &[joint_name, config] : motor_configs_)
        {
            modules.emplace(config.module_name, false);
        }

        bool all_opened = true;
        for (const auto &[module_name, _] : modules)
        {
            std::string dev_path = "/dev/" + module_name;
            int fd = openUSBCAN(dev_path.c_str());
            can_device_.set_device(module_name, fd);

            if (fd < 0)
            {
                std::cerr << module_name << " open error" << std::endl;
                all_opened = false;
            }
            else
            {
                std::cout << module_name << " open success" << std::endl;
            }
        }
        return all_opened;
    }

    const MotorConfig *motor_config(const std::string &joint_name) const
    {
        const auto it = motor_configs_.find(joint_name);
        return it == motor_configs_.end() ? nullptr : &it->second;
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
        const int ret = sendUSBCAN(fd, config.channel, &tx_msg, data);
        delay_us(140);
        return ret >= 0;
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

    // Each read sleeps in the vendor driver for up to 10 ms. Unrelated frames
    // are discarded until the deadline, so shutdown and missing motors cannot
    // leave a worker blocked forever.
    bool receive_state_blocking(const std::string &joint_name, MotorState &state,
                                int timeout_us = 10000)
    {
        const auto config_it = motor_configs_.find(joint_name);
        const auto decoder_it = decoders_.find(joint_name);
        if (config_it == motor_configs_.end() || decoder_it == decoders_.end())
        {
            return false;
        }

        const auto &config = config_it->second;
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            const auto cached = pending_states_.find(joint_name);
            if (cached != pending_states_.end())
            {
                state = cached->second;
                pending_states_.erase(cached);
                return true;
            }
        }
        if (!can_device_.is_valid(config.module_name))
        {
            return false;
        }

        const int fd = can_device_.get_device(config.module_name);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::microseconds(timeout_us);
        while (std::chrono::steady_clock::now() < deadline)
        {
            FrameInfo info_rx{};
            uint8_t channel = 0;
            uint8_t data_rx[8] = {};
            const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            const int wait_us = static_cast<int>(std::max<int64_t>(1, remaining));
            const int ret = readUSBCAN(fd, &channel, &info_rx, data_rx, wait_us);
            if (ret < 0)
            {
                return false;
            }
            auto response = decode_response(config.module_name, channel, info_rx, data_rx);
            if (!response)
            {
                continue;
            }
            if (response->first == joint_name)
            {
                state = response->second;
                return true;
            }
            {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                pending_states_[response->first] = response->second;
            }
        }
        return false;
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
        const auto near_limit = [](float value, float min, float max)
        {
            const float range = max - min;
            const float lower = min < 0.0f ? min * 0.9f : min + range * 0.1f;
            const float upper = max > 0.0f ? max * 0.9f : max - range * 0.1f;
            return value < lower || value > upper;
        };

        return near_limit(state.position, config.pos_limit[0], config.pos_limit[1]) ||
               near_limit(state.velocity, params.v_min, params.v_max) ||
               near_limit(state.electric, params.i_min, params.i_max) ||
               near_limit(command.position, config.pos_limit[0], config.pos_limit[1]) ||
               near_limit(command.velocity, params.v_min, params.v_max) ||
               near_limit(command.torque, params.t_min, params.t_max) ||
               state.temperature > 80.0f;
    }

private:
    std::unique_ptr<std::pair<std::string, MotorState>> decode_response(
        const std::string &module, uint8_t channel, const FrameInfo &info, const uint8_t data[8])
    {
        for (const auto &[joint, config] : motor_configs_)
        {
            if (config.module_name != module || config.channel != channel ||
                config.can_id != static_cast<int>(info.canID))
                continue;
            const auto decoder = decoders_.find(joint);
            if (decoder == decoders_.end() || !decoder->second->is_valid_response(data))
                return nullptr;
            MotorState state{};
            if (!decoder->second->decode(data, state))
                return nullptr;
            apply_axis_transform(state, config, false);
            return std::make_unique<std::pair<std::string, MotorState>>(joint, state);
        }
        return nullptr;
    }

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
    std::unordered_map<std::string, std::unique_ptr<MotorDecoder>> decoders_;
    std::unordered_map<std::string, MotorState> pending_states_;
    std::mutex pending_mutex_;
};
