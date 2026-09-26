#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "motor_driver/can/can_module.hpp"
#include "motor_driver/dds/dds_module.hpp"
#include "motor_driver/runtime/driver_state_machine.hpp"

class MotorDriverRuntime
{
public:
    MotorDriverRuntime(CanModule &can, DdsModule &dds, const ControlConfig &config);
    ~MotorDriverRuntime();

    bool start();
    void run();
    void stop();

private:
    struct ModuleWorker
    {
        std::string module;
        std::vector<std::string> joints;
        std::thread thread;
        std::uint64_t observed_generation = 0;
    };

    void worker_loop(size_t worker_index);
    void build_worker_schedule(const ControlConfig &config);

    CanModule &can_;
    DdsModule &dds_;
    ControlConfig config_;
    DriverStateMachine state_machine_;
    std::vector<ModuleWorker> workers_;

    std::mutex work_mutex_;
    std::condition_variable work_available_;
    std::condition_variable cycle_complete_;
    std::uint64_t generation_ = 0;
    DriverState cycle_state_ = DriverState::IDLE;
    CommandSnapshot cycle_commands_;
    std::unordered_map<std::string, MotorState> cycle_states_;
    size_t completed_workers_ = 0;
    bool cycle_safety_triggered_ = false;
    bool stopping_ = false;
    bool started_ = false;
};
