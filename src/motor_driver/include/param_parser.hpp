#pragma once

#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include "json.hpp"
#include "common.hpp"


// 电机配置解析器
class MotorConfigParser
{
public:
    MotorConfigParser() = default;

    // 从文件加载电机配置
    bool load_from_file(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open motor config file: " << file_path << std::endl;
            return false;
        }

        try {
            nlohmann::json j;
            file >> j;

            motors_.clear();

            // 遍历 JSON 对象的每个关节配置
            for (auto &[joint_name, motor_json] : j.items()) {
                MotorConfig config;
                config.module_name = motor_json.value("moddule_name", "");
                config.channel = motor_json.value("channel", 0);
                config.can_id = motor_json.value("can_id", 0);
                config.motor_name = motor_json.value("motor_name", "");
                config.axis = motor_json.value("axis", 1);
                config.offset = motor_json.value("off_set", 0.0f);

                // 解析位置限制
                if (motor_json.contains("pos_limit") && motor_json["pos_limit"].is_array()) {
                    auto limits = motor_json["pos_limit"];
                    config.pos_limit[0] = limits[0].get<float>();
                    config.pos_limit[1] = limits[1].get<float>();
                } else {
                    config.pos_limit = {-3.14f, 3.14f};  // 默认限位
                }

                motors_[joint_name] = config;
            }

            std::cout << "Loaded " << motors_.size() << " motor configurations" << std::endl;
            return true;

        } catch (const nlohmann::json::exception &e) {
            std::cerr << "JSON parse error in motor config: " << e.what() << std::endl;
            return false;
        }
    }

    // 获取所有电机配置
    const std::unordered_map<std::string, MotorConfig>& get_motors() const
    {
        return motors_;
    }

    // 根据关节名获取电机配置
    const MotorConfig* get_motor(const std::string &joint_name) const
    {
        auto it = motors_.find(joint_name);
        return (it != motors_.end()) ? &it->second : nullptr;
    }

    // 获取所有关节名称列表
    std::vector<std::string> get_joint_names() const
    {
        std::vector<std::string> names;
        names.reserve(motors_.size());
        for (const auto &[name, _] : motors_) {
            names.push_back(name);
        }
        return names;
    }

    // 检查关节是否存在
    bool has_joint(const std::string &joint_name) const
    {
        return motors_.find(joint_name) != motors_.end();
    }

    // 获取电机数量
    size_t size() const
    {
        return motors_.size();
    }

private:
    std::unordered_map<std::string, MotorConfig> motors_;
};

// 控制配置解析器
class ControlConfigParser
{
public:
    ControlConfigParser() = default;

    // 从文件加载控制配置
    bool load_from_file(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open control config file: " << file_path << std::endl;
            return false;
        }

        try {
            nlohmann::json j;
            file >> j;

            config_.control_frequency = j.value("control_frequency", 250);
            config_.command_topic = j.value("command_topic", "/joints_command");
            config_.state_topic = j.value("state_topic", "/joints_state");
            config_.frequency_info = j.value("frequency_info", 0);
            config_.joints_state_info = j.value("joints_state_info", 0);

            // 解析关节列表
            if (j.contains("joints") && j["joints"].is_array()) {
                config_.joints.clear();
                for (const auto &joint : j["joints"]) {
                    config_.joints.push_back(joint.get<std::string>());
                }
            }

            // 解析关节映射
            if (j.contains("joint_mapping") && j["joint_mapping"].is_array()) {
                config_.joint_mapping.clear();
                for (const auto &idx : j["joint_mapping"]) {
                    config_.joint_mapping.push_back(idx.get<int>());
                }
            }

            std::cout << "Loaded control config: " << config_.control_frequency
                      << " Hz, " << config_.joints.size() << " joints" << std::endl;
            return true;

        } catch (const nlohmann::json::exception &e) {
            std::cerr << "JSON parse error in control config: " << e.what() << std::endl;
            return false;
        }
    }

    // 获取控制配置
    const ControlConfig& get_config() const
    {
        return config_;
    }

    // 获取控制频率
    int get_frequency() const
    {
        return config_.control_frequency;
    }

    // 获取关节列表
    const std::vector<std::string>& get_joints() const
    {
        return config_.joints;
    }

    // 获取关节映射
    const std::vector<int>& get_joint_mapping() const
    {
        return config_.joint_mapping;
    }

private:
    ControlConfig config_;
};
