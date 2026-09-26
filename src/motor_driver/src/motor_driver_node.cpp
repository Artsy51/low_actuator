#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "can_module.hpp"
#include "dds_module.hpp"
#include "param_parser.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
enum class DriverState
{
    IDLE,
    NORMAL,
    DAMPING
};

const char *state_name(DriverState state)
{
    switch (state)
    {
    case DriverState::IDLE: return "IDLE";
    case DriverState::NORMAL: return "NORMAL";
    case DriverState::DAMPING: return "DAMPING";
    }
    return "UNKNOWN";
}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const auto package_share =
        ament_index_cpp::get_package_share_directory("motor_driver");
    MotorConfigParser motor_parser;
    ControlConfigParser control_parser;
    if (!motor_parser.load_from_file(package_share + "/config/motor.json") ||
        !control_parser.load_from_file(package_share + "/config/control.json"))
    {
        rclcpp::shutdown();
        return 1;
    }

    auto dds = std::make_shared<DdsModule>(motor_parser, control_parser);
    CanModule can(motor_parser, control_parser);
    can.init_can();

    const auto node = dds->get_node();
    const auto &joints = control_parser.get_joints();
    const int frequency = std::max(1, control_parser.get_frequency());
    const auto period = std::chrono::microseconds(1000000 / frequency);

    // Partition joints by CAN module and build a strict round-robin channel
    // schedule for each module worker.
    std::unordered_map<std::string, std::vector<std::string>> module_joints;
    for (const auto &joint : joints)
    {
        const auto config = motor_parser.get_motor(joint);
        if (config)
        {
            module_joints[config->module_name].push_back(joint);
        }
    }
    for (auto &[module, module_joint_list] : module_joints)
    {
        std::map<int, std::vector<std::string>> channel_joints;
        for (const auto &joint : module_joint_list)
        {
            const auto config = motor_parser.get_motor(joint);
            if (config)
            {
                channel_joints[config->channel].push_back(joint);
            }
        }

        std::vector<std::string> alternating;
        size_t round = 0;
        while (alternating.size() < module_joint_list.size())
        {
            bool added = false;
            for (auto &[channel, channel_list] : channel_joints)
            {
                if (round < channel_list.size())
                {
                    alternating.push_back(channel_list[round]);
                    added = true;
                }
            }
            if (!added)
            {
                break;
            }
            ++round;
        }
        module_joint_list = std::move(alternating);
    }

    DriverState state = DriverState::IDLE;
    auto last_reported_state = state;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        const bool reset_requested = dds->is_reset_requested();
        const bool command_timeout = dds->is_cmd_timeout(1000);
        const auto commands = dds->get_command_snapshot();
        std::unordered_map<std::string, MotorState> states;
        if (reset_requested || command_timeout)
        {
            state = DriverState::IDLE;
        }
        else if (state == DriverState::IDLE)
        {
            state = DriverState::NORMAL;
        }

        // IDLE sends a zero command; DAMPING sends only kd=5; NORMAL uses DDS data.
        std::mutex states_mutex;
        std::atomic<bool> limit_exceeded{false};
        std::vector<std::thread> workers;
        workers.reserve(module_joints.size());
        for (const auto &[module, module_joint_list] : module_joints)
        {
            workers.emplace_back([&, module_joint_list]()
            {
                if (module_joint_list.empty())
                {
                    return;
                }

                for (size_t offset = 0; offset < module_joint_list.size(); ++offset)
                {
                    const auto &joint = module_joint_list[offset];
                    MotorCmd command{};
                    if (state == DriverState::DAMPING)
                    {
                        command.kd = 5.0f;
                    }
                    else if (state == DriverState::NORMAL)
                    {
                        const auto command_it = commands.find(joint);
                        if (command_it != commands.end())
                        {
                            command = command_it->second;
                        }
                    }

                    can.send_command(joint, command);
                    MotorState motor_state{};
                    if (!can.receive_state(joint, motor_state))
                    {
                        continue;
                    }

                    {
                        std::lock_guard<std::mutex> lock(states_mutex);
                        states[joint] = motor_state;
                    }
                    const auto command_it = commands.find(joint);
                    const MotorCmd requested_command = command_it != commands.end()
                                                            ? command_it->second
                                                            : MotorCmd{};
                    if (can.exceeds_safety_limits(joint, motor_state, requested_command))
                    {
                        limit_exceeded.store(true);
                    }
                }
            });
        }
        for (auto &worker : workers)
        {
            worker.join();
        }
        dds->update_states(states);
        dds->publish_once();

        if (!reset_requested && !command_timeout && limit_exceeded.load())
        {
            state = DriverState::DAMPING;
        }
        else if (!reset_requested && !command_timeout && state == DriverState::DAMPING &&
                 !limit_exceeded.load())
        {
            state = DriverState::NORMAL;
        }

        if (state != last_reported_state)
        {
            RCLCPP_WARN(node->get_logger(), "Driver state: %s -> %s",
                        state_name(last_reported_state), state_name(state));
            last_reported_state = state;
        }

        std::this_thread::sleep_for(period);
    }

    rclcpp::shutdown();
    return 0;
}
