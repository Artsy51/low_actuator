#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_driver/runtime/driver_state_machine.hpp"
#include "motor_msgs/msg/joints_cmd.hpp"
#include "motor_msgs/msg/joints_data.hpp"
#include "motor_msgs/srv/disable_motor.hpp"
#include "motor_msgs/srv/enable_motor.hpp"
#include "motor_msgs/srv/reset_motor.hpp"
#include "rclcpp/rclcpp.hpp"

class DdsModule
{
public:
    explicit DdsModule(const ControlConfig &config)
        : config_(config), node_(std::make_shared<rclcpp::Node>("motor_driver"))
    {
        for (const auto &joint : config_.joints)
        {
            commands_.emplace(joint, MotorCmd{});
            states_.emplace(joint, MotorState{});
        }

        command_subscription_ = node_->create_subscription<motor_msgs::msg::JointsCmd>(
            config_.command_topic, rclcpp::SensorDataQoS(),
            [this](motor_msgs::msg::JointsCmd::ConstSharedPtr message)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                const size_t count = std::min(config_.joints.size(), message->data.size());
                for (size_t i = 0; i < count; ++i)
                {
                    const auto &value = message->data[i];
                    commands_[config_.joints[i]] = MotorCmd{
                        value.position, value.velocity, value.torque, value.kp, value.kd};
                }
                ++command_sequence_;
                last_command_time_ = std::chrono::steady_clock::now();
                has_received_command_ = true;
            });

        state_publisher_ = node_->create_publisher<motor_msgs::msg::JointsData>(
            config_.state_topic, 10);

        reset_service_ = node_->create_service<motor_msgs::srv::ResetMotor>(
            "/motor_driver/reset",
            [this](const std::shared_ptr<motor_msgs::srv::ResetMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::ResetMotor::Response> response)
            {
                reset_requested_.store(true);
                response->success = true;
                response->message = "Driver reset to idle; waiting for a new command";
            });

        enable_service_ = node_->create_service<motor_msgs::srv::EnableMotor>(
            "/motor_driver/enable",
            [this](const std::shared_ptr<motor_msgs::srv::EnableMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::EnableMotor::Response> response)
            {
                enable_requested_.store(true);
                response->success = true;
                response->message = "Driver enabled";
            });

        disable_service_ = node_->create_service<motor_msgs::srv::DisableMotor>(
            "/motor_driver/disable",
            [this](const std::shared_ptr<motor_msgs::srv::DisableMotor::Request>,
                   std::shared_ptr<motor_msgs::srv::DisableMotor::Response> response)
            {
                disable_requested_.store(true);
                response->success = true;
                response->message = "Driver disabled";
            });
    }

    rclcpp::Node::SharedPtr node() const
    {
        return node_;
    }

    CommandSnapshot command_snapshot(std::chrono::milliseconds timeout) const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const bool timed_out = !has_received_command_ ||
            std::chrono::steady_clock::now() - last_command_time_ > timeout;
        return CommandSnapshot{commands_, command_sequence_, timed_out};
    }

    DriverEvents consume_events()
    {
        return DriverEvents{
            reset_requested_.exchange(false),
            enable_requested_.exchange(false),
            disable_requested_.exchange(false)};
    }

    // Apply only newly received CAN states and count those actual updates.
    void update_states(const std::unordered_map<std::string, MotorState> &updates)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (const auto &[joint, state] : updates)
            {
                const auto it = states_.find(joint);
                if (it != states_.end())
                {
                    const bool changed = !state_valid_[joint] ||
                        it->second.position != state.position ||
                        it->second.velocity != state.velocity ||
                        it->second.electric != state.electric ||
                        it->second.temperature != state.temperature;
                    if (changed)
                    {
                        it->second = state;
                        state_valid_[joint] = true;
                        ++update_counts_[joint];
                    }
                }
            }
        }
    }

    void publish_states()
    {
        motor_msgs::msg::JointsData message;
        message.stamp = node_->now();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            message.data.reserve(config_.joints.size());
            for (const auto &joint : config_.joints)
            {
                const auto &state = states_.at(joint);
                motor_msgs::msg::JointDataValue value;
                value.position = state.position;
                value.velocity = state.velocity;
                value.electric = state.electric;
                value.temperature = state.temperature;
                message.data.push_back(value);
            }
        }
        state_publisher_->publish(message);
        output_diagnostics_if_needed();
    }

private:
    void output_diagnostics_if_needed()
    {
        const auto now = std::chrono::steady_clock::now();
        std::unordered_map<std::string, std::size_t> counts;
        std::unordered_map<std::string, MotorState> states;
        double elapsed_seconds = 0.0;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const auto elapsed = now - diagnostics_last_output_;
            // Diagnostics use a fixed three-second measurement and output window.
            if (elapsed < std::chrono::seconds(3) ||
                (config_.frequency_info == 0 && config_.joints_state_info == 0))
            {
                return;
            }
            elapsed_seconds = std::chrono::duration<double>(elapsed).count();
            counts = update_counts_;
            states = states_;
            update_counts_.clear();
            diagnostics_last_output_ = now;
        }

        if (config_.frequency_info != 0)
        {
            std::ostringstream output;
            output << "\n===== Joint Update Frequency (Hz) =====\n";
            output << std::fixed << std::setprecision(1);
            for (const auto &joint : config_.joints)
            {
                output << "  " << joint << ": "
                       << static_cast<double>(counts[joint]) / elapsed_seconds << " Hz\n";
            }
            output << "=======================================\n";
            RCLCPP_INFO(node_->get_logger(), "%s", output.str().c_str());
        }

        if (config_.joints_state_info != 0)
        {
            std::ostringstream output;
            output << "\n===== Joint State =====\n";
            output << std::left << std::setw(24) << "Joint"
                   << std::right << std::setw(10) << "Position"
                   << std::setw(10) << "Velocity"
                   << std::setw(10) << "Current"
                   << std::setw(10) << "Temp" << "\n";
            output << std::fixed << std::setprecision(3);
            for (const auto &joint : config_.joints)
            {
                const auto &state = states.at(joint);
                output << std::left << std::setw(24) << joint
                       << std::right << std::setw(10) << state.position
                       << std::setw(10) << state.velocity
                       << std::setw(10) << state.electric
                       << std::setw(10) << state.temperature << "\n";
            }
            output << "=======================\n";
            RCLCPP_INFO(node_->get_logger(), "%s", output.str().c_str());
        }
    }

    ControlConfig config_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<motor_msgs::msg::JointsCmd>::SharedPtr command_subscription_;
    rclcpp::Publisher<motor_msgs::msg::JointsData>::SharedPtr state_publisher_;
    rclcpp::Service<motor_msgs::srv::ResetMotor>::SharedPtr reset_service_;
    rclcpp::Service<motor_msgs::srv::EnableMotor>::SharedPtr enable_service_;
    rclcpp::Service<motor_msgs::srv::DisableMotor>::SharedPtr disable_service_;

    mutable std::mutex data_mutex_;
    std::unordered_map<std::string, MotorCmd> commands_;
    std::unordered_map<std::string, MotorState> states_;
    std::unordered_map<std::string, bool> state_valid_;
    std::unordered_map<std::string, std::size_t> update_counts_;
    std::uint64_t command_sequence_ = 0;
    bool has_received_command_ = false;
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point diagnostics_last_output_{std::chrono::steady_clock::now()};
    std::atomic<bool> reset_requested_{false};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> disable_requested_{false};
};
