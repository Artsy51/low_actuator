#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "motor_driver/core/common.hpp"

enum class DriverState
{
    IDLE,
    NORMAL,
    DAMPING,
    DISABLED
};

struct DriverEvents
{
    bool reset = false;
    bool enable = false;
    bool disable = false;
};

struct CommandSnapshot
{
    std::unordered_map<std::string, MotorCmd> commands;
    std::uint64_t sequence = 0;
    bool timed_out = true;
};

class DriverStateMachine
{
public:
    DriverState update(const CommandSnapshot &snapshot,
                       const DriverEvents &events,
                       bool safety_triggered)
    {
        if (events.disable)
        {
            state_ = DriverState::DISABLED;
            return state_;
        }
        if (events.reset)
        {
            state_ = DriverState::IDLE;
            command_sequence_at_reset_ = snapshot.sequence;
            return state_;
        }
        if (events.enable)
        {
            state_ = snapshot.timed_out ? DriverState::IDLE : DriverState::NORMAL;
            return state_;
        }
        if (state_ == DriverState::DISABLED)
        {
            return state_;
        }
        if (snapshot.timed_out)
        {
            state_ = DriverState::IDLE;
            return state_;
        }
        if (safety_triggered)
        {
            state_ = DriverState::DAMPING;
            return state_;
        }
        if (state_ == DriverState::DAMPING)
        {
            // Keep the protective damping command until an explicit enable or
            // reset request changes the mode.
            return state_;
        }
        if (state_ == DriverState::IDLE &&
                 snapshot.sequence > command_sequence_at_reset_)
        {
            state_ = DriverState::NORMAL;
        }
        return state_;
    }

    MotorCmd command_for(DriverState state,
                         const std::string &joint,
                         const CommandSnapshot &snapshot) const
    {
        MotorCmd command{};
        if (state == DriverState::DAMPING)
        {
            command.kd = 5.0f;
        }
        else if (state == DriverState::NORMAL)
        {
            const auto it = snapshot.commands.find(joint);
            if (it != snapshot.commands.end())
            {
                command = it->second;
            }
        }
        return command;
    }

    DriverState state() const
    {
        return state_;
    }

    static const char *name(DriverState state)
    {
        switch (state)
        {
        case DriverState::IDLE: return "IDLE";
        case DriverState::NORMAL: return "NORMAL";
        case DriverState::DAMPING: return "DAMPING";
        case DriverState::DISABLED: return "DISABLED";
        }
        return "UNKNOWN";
    }

private:
    DriverState state_ = DriverState::IDLE;
    std::uint64_t command_sequence_at_reset_ = 0;
};
