#pragma once

#include <array>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "motor_driver/core/common.hpp"
#include "json.hpp"
#include "motor_driver/can/motor_decoder.hpp"

class MotorConfigParser
{
public:
    bool load_from_file(const std::string &file_path)
    {
        try
        {
            std::ifstream file(file_path);
            if (!file)
            {
                std::cerr << "Cannot open motor config: " << file_path << std::endl;
                return false;
            }

            const auto json = nlohmann::json::parse(file);
            if (!json.is_object() || json.empty())
            {
                std::cerr << "Motor config must be a non-empty JSON object" << std::endl;
                return false;
            }

            std::unordered_map<std::string, MotorConfig> parsed;
            std::set<std::tuple<std::string, int, int>> endpoints;
            for (const auto &[joint, value] : json.items())
            {
                if (!value.is_object())
                {
                    std::cerr << "Motor config for " << joint << " must be an object" << std::endl;
                    return false;
                }

                MotorConfig config{};
                if (value.contains("module_name"))
                {
                    config.module_name = value.at("module_name").get<std::string>();
                }
                else if (value.contains("moddule_name"))
                {
                    config.module_name = value.at("moddule_name").get<std::string>();
                    std::cerr << "Warning: 'moddule_name' is deprecated; use 'module_name'"
                              << std::endl;
                }
                config.channel = value.at("channel").get<int>();
                config.can_id = value.at("can_id").get<int>();
                config.motor_name = value.at("motor_name").get<std::string>();
                config.axis = value.at("axis").get<int>();
                config.offset = value.value("offset", value.value("off_set", 0.0f));

                if (!value.contains("pos_limit") || !value.at("pos_limit").is_array() ||
                    value.at("pos_limit").size() != 2)
                {
                    std::cerr << "Motor " << joint << " requires two pos_limit values" << std::endl;
                    return false;
                }
                config.pos_limit = {value.at("pos_limit")[0].get<float>(),
                                    value.at("pos_limit")[1].get<float>()};
                if (joint.empty() || config.module_name.empty() || config.channel < 0 ||
                    config.can_id < 0 || config.motor_name.empty() ||
                    (config.axis != 1 && config.axis != -1) ||
                    config.pos_limit[0] >= config.pos_limit[1])
                {
                    std::cerr << "Invalid motor configuration for joint " << joint << std::endl;
                    return false;
                }
                if (!endpoints.emplace(config.module_name, config.channel, config.can_id).second)
                {
                    std::cerr << "Duplicate CAN endpoint for joint " << joint << std::endl;
                    return false;
                }
                if (!MotorDecoderFactory::is_supported(config.motor_name))
                {
                    std::cerr << "Unsupported motor model '" << config.motor_name
                              << "' for joint " << joint << std::endl;
                    return false;
                }
                parsed.emplace(joint, std::move(config));
            }
            motors_ = std::move(parsed);
            return true;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Invalid motor config " << file_path << ": " << error.what() << std::endl;
            return false;
        }
    }

    const std::unordered_map<std::string, MotorConfig> &get_motors() const { return motors_; }

    const MotorConfig *get_motor(const std::string &joint) const
    {
        const auto it = motors_.find(joint);
        return it == motors_.end() ? nullptr : &it->second;
    }

    std::vector<std::string> get_joint_names() const
    {
        std::vector<std::string> names;
        names.reserve(motors_.size());
        for (const auto &[name, config] : motors_) names.push_back(name);
        return names;
    }

    bool has_joint(const std::string &joint) const { return motors_.count(joint) != 0; }
    size_t size() const { return motors_.size(); }

private:
    std::unordered_map<std::string, MotorConfig> motors_;
};

class ControlConfigParser
{
public:
    bool load_from_file(const std::string &file_path)
    {
        try
        {
            std::ifstream file(file_path);
            if (!file)
            {
                std::cerr << "Cannot open control config: " << file_path << std::endl;
                return false;
            }

            const auto json = nlohmann::json::parse(file);
            if (!json.is_object())
            {
                std::cerr << "Control config must be a JSON object" << std::endl;
                return false;
            }

            ControlConfig parsed{};
            parsed.control_frequency = json.value("control_frequency", 250);
            parsed.command_topic = json.value("command_topic", std::string("/joints_command"));
            parsed.state_topic = json.value("state_topic", std::string("/joints_state"));
            parsed.frequency_info = json.value("frequency_info", 0);
            parsed.joints_state_info = json.value("joints_state_info", 0);
            parsed.joints = json.at("joints").get<std::vector<std::string>>();
            parsed.joint_mapping = json.value("joint_mapping", std::vector<int>{});

            if (parsed.control_frequency <= 0 || parsed.command_topic.empty() ||
                parsed.state_topic.empty() || parsed.joints.empty())
            {
                std::cerr << "Control frequency, topics and joints must be valid" << std::endl;
                return false;
            }
            if (parsed.joint_mapping.empty())
            {
                parsed.joint_mapping.resize(parsed.joints.size());
                for (size_t i = 0; i < parsed.joints.size(); ++i)
                    parsed.joint_mapping[i] = static_cast<int>(i);
            }
            if (parsed.joint_mapping.size() != parsed.joints.size())
            {
                std::cerr << "joint_mapping length must match joints length" << std::endl;
                return false;
            }

            std::set<std::string> unique_joints;
            std::set<int> unique_indices;
            for (size_t i = 0; i < parsed.joints.size(); ++i)
            {
                if (parsed.joints[i].empty() || !unique_joints.insert(parsed.joints[i]).second ||
                    parsed.joint_mapping[i] < 0 ||
                    static_cast<size_t>(parsed.joint_mapping[i]) >= parsed.joints.size() ||
                    !unique_indices.insert(parsed.joint_mapping[i]).second)
                {
                    std::cerr << "Joints and joint_mapping must contain unique valid entries"
                              << std::endl;
                    return false;
                }
            }
            config_ = std::move(parsed);
            return true;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Invalid control config " << file_path << ": " << error.what()
                      << std::endl;
            return false;
        }
    }

    const ControlConfig &get_config() const { return config_; }
    int get_frequency() const { return config_.control_frequency; }
    const std::vector<std::string> &get_joints() const { return config_.joints; }
    const std::vector<int> &get_joint_mapping() const { return config_.joint_mapping; }

private:
    ControlConfig config_{};
};
