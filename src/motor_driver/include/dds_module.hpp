#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motor_msgs/msg/joints_cmd.hpp"
#include "motor_msgs/msg/joints_data.hpp"
#include "motor_msgs/srv/disable_motor.hpp"
#include "motor_msgs/srv/enable_motor.hpp"
#include "motor_msgs/srv/reset_motor.hpp"

#include "common.hpp"
#include "param_parser.hpp"

// DDS 通信模块 - 负责 ROS2 话题订阅/发布和服务处理
class DdsModule
{
public:
    // 控制模式
    enum class Mode
    {
        DISABLED,  // 失能模式
        ENABLED,   // 使能模式
        PROTECT,   // 保护模式
    };

    explicit DdsModule(const MotorConfigParser &motor_parser, const ControlConfigParser &ctrl_parser)
        : motor_configs_(motor_parser.get_motors()),
          ctrl_config_(ctrl_parser.get_config())
    {
        // 创建 ROS2 节点
        node_ = std::make_shared<rclcpp::Node>("motor_driver_dds");

        size_t num_joints = ctrl_config_.joints.size();

        // 初始化缓冲区
        cmd_buffer_.resize(num_joints);
        state_buffer_.resize(num_joints);

        // 订阅指令话题
        cmd_sub_ = node_->create_subscription<motor_msgs::msg::JointsCmd>(
            ctrl_config_.command_topic, 10,
            [this](const motor_msgs::msg::JointsCmd::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                const size_t update_count = std::min({msg->data.size(), cmd_buffer_.size(),
                                                      ctrl_config_.joint_mapping.size()});
                for (size_t i = 0; i < update_count; i++)
                {
                    int idx = ctrl_config_.joint_mapping[i];
                    if (idx < 0 || static_cast<size_t>(idx) >= cmd_buffer_.size())
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                                   "Invalid joint_mapping[%zu]=%d for command buffer size %zu",
                                   i, idx, cmd_buffer_.size());
                        continue;
                    }
                    cmd_buffer_[idx].position = msg->data[i].position;
                    cmd_buffer_[idx].velocity = msg->data[i].velocity;
                    cmd_buffer_[idx].torque = msg->data[i].torque;
                    cmd_buffer_[idx].kp = msg->data[i].kp;
                    cmd_buffer_[idx].kd = msg->data[i].kd;
                }
                check_command_safety();
                cmd_received_.store(true);
                last_cmd_time_ = std::chrono::steady_clock::now();
            });

        // 创建状态发布者
        state_pub_ = node_->create_publisher<motor_msgs::msg::JointsData>(
            ctrl_config_.state_topic, 10);

        // 创建服务
        disable_srv_ = node_->create_service<motor_msgs::srv::DisableMotor>(
            "/motor_driver/disable",
            [this](const std::shared_ptr<motor_msgs::srv::DisableMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::DisableMotor::Response> response)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_mode_ = Mode::DISABLED;
                response->success = true;
                response->message = "Motor driver disabled";
                RCLCPP_INFO(node_->get_logger(), "Motor driver disabled");
            });

        enable_srv_ = node_->create_service<motor_msgs::srv::EnableMotor>(
            "/motor_driver/enable",
            [this](const std::shared_ptr<motor_msgs::srv::EnableMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::EnableMotor::Response> response)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_mode_ = Mode::ENABLED;
                protect_reason_.clear();
                response->success = true;
                response->message = "Motor driver enabled";
                RCLCPP_INFO(node_->get_logger(), "Motor driver enabled");
            });

        reset_srv_ = node_->create_service<motor_msgs::srv::ResetMotor>(
            "/motor_driver/reset",
            [this](const std::shared_ptr<motor_msgs::srv::ResetMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::ResetMotor::Response> response)
            {
                // Reset 操作需要通过 CAN 发送置零命令
                // 这里只标记请求，具体实现在主循环中处理
                reset_requested_.store(true);
                response->success = true;
                response->message = "Reset command queued";
                RCLCPP_INFO(node_->get_logger(), "Reset command queued");
            });

        // 创建定时器（初始停止）
        auto period = std::chrono::milliseconds(1000 / ctrl_config_.control_frequency);
        timer_ = node_->create_wall_timer(period, [this]() { publish_state(); });
        timer_->cancel();
    }

    ~DdsModule() = default;

    // 获取 ROS2 节点
    rclcpp::Node::SharedPtr get_node()
    {
        return node_;
    }

    // 启动状态发布定时器
    void start_publish()
    {
        timer_->reset();
    }

    // 停止状态发布定时器
    void stop_publish()
    {
        timer_->cancel();
    }

    // 手动发布一次状态
    void publish_once()
    {
        publish_state();
    }

    // 检查是否有新指令到达（检查后自动清除标志）
    bool has_cmd_received()
    {
        return cmd_received_.exchange(false);
    }

    // 检查指令是否超时
    bool is_cmd_timeout(int timeout_ms = 1000)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_cmd_time_).count();
        return elapsed > timeout_ms;
    }

    // Return the most recent DDS command without applying the state machine mode.
    std::unordered_map<std::string, MotorCmd> get_command_snapshot() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::unordered_map<std::string, MotorCmd> cmds;
        const size_t count = std::min(ctrl_config_.joints.size(),
                                      ctrl_config_.joint_mapping.size());
        for (size_t i = 0; i < count; ++i)
        {
            const int idx = ctrl_config_.joint_mapping[i];
            if (idx >= 0 && static_cast<size_t>(idx) < cmd_buffer_.size())
            {
                cmds[ctrl_config_.joints[i]] = cmd_buffer_[idx];
            }
        }
        return cmds;
    }

    // 检查是否请求了复位
    bool is_reset_requested()
    {
        return reset_requested_.exchange(false);
    }

    // 获取指令数据快照（按关节名称索引）
    std::unordered_map<std::string, MotorCmd> get_cmds()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::unordered_map<std::string, MotorCmd> cmds;

        if (current_mode_ != Mode::ENABLED)
        {
            // 非使能模式，返回零力矩保持指令
            for (size_t i = 0; i < ctrl_config_.joints.size(); i++)
            {
                MotorCmd zero_cmd{};
                zero_cmd.position = state_buffer_[i].position;  // 保持当前位置
                zero_cmd.velocity = 0.0f;
                zero_cmd.torque = 0.0f;
                zero_cmd.kp = 0.0f;
                zero_cmd.kd = 5.0f;
                cmds[ctrl_config_.joints[i]] = zero_cmd;
            }
        }
        else
        {
            for (size_t i = 0; i < ctrl_config_.joints.size(); i++)
            {
                int idx = ctrl_config_.joint_mapping[i];
                if (idx >= 0 && static_cast<size_t>(idx) < cmd_buffer_.size())
                {
                    cmds[ctrl_config_.joints[i]] = cmd_buffer_[idx];
                }
            }
        }

        return cmds;
    }

    // 更新状态数据（按关节名称索引）
    void update_states(const std::unordered_map<std::string, MotorState> &states)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        for (size_t i = 0; i < ctrl_config_.joints.size(); i++)
        {
            const auto &joint_name = ctrl_config_.joints[i];
            auto it = states.find(joint_name);
            if (it != states.end())
            {
                int idx = ctrl_config_.joint_mapping[i];
                if (idx >= 0 && static_cast<size_t>(idx) < state_buffer_.size())
                {
                    state_buffer_[idx] = it->second;
                }
            }
        }

    }

    // 获取当前模式
    Mode get_mode() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return current_mode_;
    }

    // 获取保护原因
    std::string get_protect_reason() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return protect_reason_;
    }

    void set_mode(Mode mode)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_mode_ = mode;
        if (mode != Mode::PROTECT)
        {
            protect_reason_.clear();
        }
    }

private:
    // 发布状态数据
    void publish_state()
    {
        auto msg = motor_msgs::msg::JointsData();
        msg.stamp = node_->now();

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const size_t publish_count = std::min(ctrl_config_.joints.size(),
                                                  ctrl_config_.joint_mapping.size());
            for (size_t i = 0; i < publish_count; i++)
            {
                int idx = ctrl_config_.joint_mapping[i];
                if (idx < 0 || static_cast<size_t>(idx) >= state_buffer_.size())
                {
                    RCLCPP_ERROR(node_->get_logger(),
                               "Invalid joint_mapping[%zu]=%d for state buffer size %zu",
                               i, idx, state_buffer_.size());
                    continue;
                }

                motor_msgs::msg::JointDataValue data;
                data.position = state_buffer_[idx].position;
                data.velocity = state_buffer_[idx].velocity;
                data.electric = state_buffer_[idx].electric;
                data.temperature = state_buffer_[idx].temperature;
                msg.data.push_back(data);
            }
        }

        state_pub_->publish(msg);
    }

    // 检查指令安全性
    bool check_command_safety()
    {
        for (size_t i = 0; i < ctrl_config_.joints.size(); i++)
        {
            const auto &joint_name = ctrl_config_.joints[i];
            auto config_it = motor_configs_.find(joint_name);
            if (config_it == motor_configs_.end())
            {
                continue;
            }
            const auto &config = config_it->second;

            int idx = ctrl_config_.joint_mapping[i];
            if (idx < 0 || static_cast<size_t>(idx) >= cmd_buffer_.size())
            {
                continue;
            }

            // 检查位置指令超限
            if (cmd_buffer_[idx].position < config.pos_limit[0] ||
                cmd_buffer_[idx].position > config.pos_limit[1])
            {
                set_protect("Command position out of limit on joint " + joint_name);
                return false;
            }
        }

        return true;
    }

    // 设置保护模式
    void set_protect(const std::string &reason)
    {
        current_mode_ = Mode::PROTECT;
        protect_reason_ = reason;
        RCLCPP_WARN(node_->get_logger(), "Protection triggered: %s", reason.c_str());
    }

private:
    // ROS2 组件
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<motor_msgs::msg::JointsCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<motor_msgs::msg::JointsData>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 服务
    rclcpp::Service<motor_msgs::srv::DisableMotor>::SharedPtr disable_srv_;
    rclcpp::Service<motor_msgs::srv::EnableMotor>::SharedPtr enable_srv_;
    rclcpp::Service<motor_msgs::srv::ResetMotor>::SharedPtr reset_srv_;

    // 数据缓冲区
    std::vector<MotorCmd> cmd_buffer_;
    std::vector<MotorState> state_buffer_;
    mutable std::mutex data_mutex_;

    // 配置参数
    std::unordered_map<std::string, MotorConfig> motor_configs_;
    ControlConfig ctrl_config_;

    // 安全状态
    Mode current_mode_ = Mode::DISABLED;
    std::string protect_reason_;

    // 指令和复位标志
    std::atomic<bool> cmd_received_{false};
    std::atomic<bool> reset_requested_{false};
    std::chrono::steady_clock::time_point last_cmd_time_;
};
