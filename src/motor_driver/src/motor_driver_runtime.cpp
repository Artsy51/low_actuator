#include "motor_driver/runtime/motor_driver_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>

MotorDriverRuntime::MotorDriverRuntime(CanModule &can, DdsModule &dds,
                                       const ControlConfig &config)
    : can_(can), dds_(dds), config_(config)
{
    build_worker_schedule(config_);
}

MotorDriverRuntime::~MotorDriverRuntime()
{
    stop();
}

void MotorDriverRuntime::build_worker_schedule(const ControlConfig &config)
{
    std::map<std::string, std::map<int, std::vector<std::string>>> grouped;
    for (const auto &joint : config.joints)
    {
        const auto *motor = can_.motor_config(joint);
        if (motor != nullptr)
        {
            grouped[motor->module_name][motor->channel].push_back(joint);
        }
    }

    for (const auto &[module, channels] : grouped)
    {
        ModuleWorker worker;
        worker.module = module;
        size_t round = 0;
        bool added = true;
        while (added)
        {
            added = false;
            for (const auto &[channel, joints] : channels)
            {
                if (round < joints.size())
                {
                    worker.joints.push_back(joints[round]);
                    added = true;
                }
            }
            ++round;
        }
        workers_.push_back(std::move(worker));
    }
}

bool MotorDriverRuntime::start()
{
    if (started_)
    {
        return true;
    }
    if (workers_.empty())
    {
        RCLCPP_ERROR(dds_.node()->get_logger(), "No CAN modules are configured");
        return false;
    }
    if (!can_.init_can())
    {
        RCLCPP_ERROR(dds_.node()->get_logger(), "Failed to open one or more CAN modules");
        return false;
    }

    stopping_ = false;
    started_ = true;
    for (size_t i = 0; i < workers_.size(); ++i)
    {
        workers_[i].thread = std::thread(&MotorDriverRuntime::worker_loop, this, i);
    }
    return true;
}

void MotorDriverRuntime::run()
{
    if (!started_ && !start())
    {
        return;
    }

    const auto period = std::chrono::microseconds(
        1000000 / std::max(1, config_.control_frequency));
    auto next_cycle = std::chrono::steady_clock::now();
    bool safety_triggered = false;
    DriverState reported_state = state_machine_.state();

    while (rclcpp::ok())
    {
        const auto snapshot = dds_.command_snapshot(std::chrono::milliseconds(1000));
        const auto events = dds_.consume_events();
        const DriverState state = state_machine_.update(snapshot, events, safety_triggered);

        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            cycle_state_ = state;
            cycle_commands_ = snapshot;
            cycle_states_.clear();
            cycle_safety_triggered_ = false;
            completed_workers_ = 0;
            ++generation_;
        }
        work_available_.notify_all();

        {
            std::unique_lock<std::mutex> lock(work_mutex_);
            cycle_complete_.wait(lock, [this]
            {
                return stopping_ || completed_workers_ == workers_.size();
            });
            if (stopping_)
            {
                break;
            }
            safety_triggered = cycle_safety_triggered_;
            auto states = cycle_states_;
            lock.unlock();
            dds_.update_states(states);
            dds_.publish_states();
        }

        if (state != reported_state)
        {
            RCLCPP_WARN(dds_.node()->get_logger(), "Driver state: %s -> %s",
                        DriverStateMachine::name(reported_state),
                        DriverStateMachine::name(state));
            reported_state = state;
        }

        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);
        if (std::chrono::steady_clock::now() - next_cycle > period)
        {
            next_cycle = std::chrono::steady_clock::now();
        }
    }
    stop();
}

void MotorDriverRuntime::stop()
{
    {
        std::lock_guard<std::mutex> lock(work_mutex_);
        stopping_ = true;
    }
    work_available_.notify_all();
    cycle_complete_.notify_all();
    for (auto &worker : workers_)
    {
        if (worker.thread.joinable())
        {
            worker.thread.join();
        }
    }
    started_ = false;
}

void MotorDriverRuntime::worker_loop(size_t worker_index)
{
    auto &worker = workers_[worker_index];
    while (true)
    {
        DriverState state;
        CommandSnapshot commands;
        std::uint64_t generation;
        {
            std::unique_lock<std::mutex> lock(work_mutex_);
            work_available_.wait(lock, [&]
            {
                return stopping_ || generation_ > worker.observed_generation;
            });
            if (stopping_)
            {
                return;
            }
            generation = generation_;
            worker.observed_generation = generation;
            state = cycle_state_;
            commands = cycle_commands_;
        }

        std::unordered_map<std::string, MotorState> states;
        bool safety_triggered = false;
        for (const auto &joint : worker.joints)
        {
            const MotorCmd command = state_machine_.command_for(state, joint, commands);
            if (!can_.send_command(joint, command))
            {
                safety_triggered = true;
                RCLCPP_ERROR(dds_.node()->get_logger(),
                             "Failed to send CAN command for joint %s", joint.c_str());
                continue;
            }

            MotorState motor_state{};
            if (!can_.receive_state_blocking(joint, motor_state))
            {
                safety_triggered = true;
                RCLCPP_WARN(dds_.node()->get_logger(),
                            "No valid CAN response for joint %s", joint.c_str());
                continue;
            }
            states[joint] = motor_state;
            safety_triggered = safety_triggered ||
                can_.exceeds_safety_limits(joint, motor_state, command);
        }

        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            if (generation == generation_)
            {
                cycle_states_.insert(states.begin(), states.end());
                cycle_safety_triggered_ = cycle_safety_triggered_ || safety_triggered;
                ++completed_workers_;
            }
        }
        cycle_complete_.notify_one();
    }
}
