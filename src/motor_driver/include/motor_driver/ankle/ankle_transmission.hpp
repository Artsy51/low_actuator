#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_driver/ankle/ankle_solver.hpp"
#include "motor_driver/core/common.hpp"

// 把话题上的踝端 pitch/roll 和 CAN 上的两对并联电机互相换算。
// 左腿话题 roll 与求解器 roll 反号，右腿同号。求解器内部顺序是 [roll, pitch]。
class AnkleTransmission
{
public:
    static constexpr std::array<const char *, 4> kJointNames{
        "Left_calf_left", "Left_calf_right", "Right_calf_left", "Right_calf_right"};
    static constexpr double kPitchMin = -0.873;
    static constexpr double kPitchMax = 0.523;
    static constexpr double kRollMin = -0.262;
    static constexpr double kRollMax = 0.262;

    AnkleTransmission()
    {
        bind(legs_[0], "Left_calf_left", "Left_calf_right", -1.0);
        bind(legs_[1], "Right_calf_left", "Right_calf_right", 1.0);
    }

    static bool joints_available(const std::vector<std::string> &joints)
    {
        for (const char *name : kJointNames)
        {
            if (std::find(joints.begin(), joints.end(), name) == joints.end())
            {
                return false;
            }
        }
        return true;
    }

    // 仅改四个 calf 槽位。失败的那条腿保持上一帧电机指令。
    bool apply_command(std::unordered_map<std::string, MotorCmd> &commands)
    {
        bool ok = true;
        for (auto &leg : legs_)
        {
            ok = apply_leg_command(leg, commands) && ok;
        }
        return ok;
    }

    // 四个 calf 槽位换成踝端 pitch/roll，电流和温度保持电机量。
    void apply_state(std::unordered_map<std::string, MotorState> &states)
    {
        for (auto &leg : legs_)
        {
            apply_leg_state(leg, states);
        }
    }

private:
    struct Leg
    {
        std::string pitch_joint;
        std::string roll_joint;
        double roll_sign = 1.0;
        Ankle_Solver solver;
        Eigen::Vector2d zero = Eigen::Vector2d::Zero();
        Eigen::Vector2d pose = Eigen::Vector2d::Zero();
        bool pose_valid = false;
        MotorCmd last_pitch{};
        MotorCmd last_roll{};
        bool has_last_command = false;
    };

    static void bind(Leg &leg, const char *pitch, const char *roll, double roll_sign)
    {
        leg.pitch_joint = pitch;
        leg.roll_joint = roll;
        leg.roll_sign = roll_sign;
        leg.zero = leg.solver.ik_pose(0.0, 0.0);
        if (!leg.zero.allFinite())
        {
            throw std::runtime_error("Ankle zero pose is not finite");
        }
    }

    static double clamp(double value, double lower, double upper)
    {
        return std::min(upper, std::max(lower, value));
    }

    static void hold(Leg &leg, std::unordered_map<std::string, MotorCmd> &commands)
    {
        if (leg.has_last_command)
        {
            commands[leg.pitch_joint] = leg.last_pitch;
            commands[leg.roll_joint] = leg.last_roll;
            return;
        }
        commands[leg.pitch_joint] = MotorCmd{};
        commands[leg.roll_joint] = MotorCmd{};
    }

    bool apply_leg_command(Leg &leg, std::unordered_map<std::string, MotorCmd> &commands)
    {
        const auto pitch_it = commands.find(leg.pitch_joint);
        const auto roll_it = commands.find(leg.roll_joint);
        if (pitch_it == commands.end() || roll_it == commands.end())
        {
            return false;
        }

        MotorCmd pitch = pitch_it->second;
        MotorCmd roll = roll_it->second;
        if (!std::isfinite(pitch.position) || !std::isfinite(pitch.velocity) ||
            !std::isfinite(pitch.torque) || !std::isfinite(roll.position) ||
            !std::isfinite(roll.velocity) || !std::isfinite(roll.torque))
        {
            hold(leg, commands);
            return false;
        }

        const double pitch_pos = clamp(pitch.position, kPitchMin, kPitchMax);
        const double roll_pos = clamp(roll.position, kRollMin, kRollMax);
        const Eigen::Vector2d solver_pose(leg.roll_sign * roll_pos, pitch_pos);
        const Eigen::Vector2d motor = leg.solver.ik_pose(solver_pose[0], solver_pose[1]);
        const Eigen::Vector2d jac_pose = leg.pose_valid ? leg.pose : solver_pose;
        const Eigen::Matrix2d jacobian = leg.solver.jacobian(jac_pose[0], jac_pose[1]);
        const Eigen::Vector2d solver_velocity(leg.roll_sign * roll.velocity,
                                              pitch.velocity);
        const Eigen::Vector2d solver_torque(leg.roll_sign * roll.torque, pitch.torque);
        const Eigen::Vector2d motor_velocity =
            jacobian.allFinite() ? Eigen::Vector2d(jacobian * solver_velocity)
                                 : Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
        const Eigen::Vector2d motor_torque = leg.solver.ik_torque(solver_torque, jac_pose);
        if (!motor.allFinite() || !motor_velocity.allFinite() || !motor_torque.allFinite())
        {
            hold(leg, commands);
            return false;
        }

        pitch.position = static_cast<float>(motor[0] - leg.zero[0]);
        roll.position = static_cast<float>(motor[1] - leg.zero[1]);
        pitch.velocity = static_cast<float>(motor_velocity[0]);
        roll.velocity = static_cast<float>(motor_velocity[1]);
        pitch.torque = static_cast<float>(motor_torque[0]);
        roll.torque = static_cast<float>(motor_torque[1]);
        commands[leg.pitch_joint] = pitch;
        commands[leg.roll_joint] = roll;
        leg.last_pitch = pitch;
        leg.last_roll = roll;
        leg.has_last_command = true;
        return true;
    }

    void apply_leg_state(Leg &leg, std::unordered_map<std::string, MotorState> &states)
    {
        auto pitch_it = states.find(leg.pitch_joint);
        auto roll_it = states.find(leg.roll_joint);
        if (pitch_it == states.end() || roll_it == states.end())
        {
            return;
        }

        auto publish_hold = [&]()
        {
            pitch_it->second.position = leg.pose_valid ? static_cast<float>(leg.pose[1]) : 0.0f;
            roll_it->second.position =
                leg.pose_valid ? static_cast<float>(leg.roll_sign * leg.pose[0]) : 0.0f;
            pitch_it->second.velocity = 0.0f;
            roll_it->second.velocity = 0.0f;
        };

        const Eigen::Vector2d motor_position(pitch_it->second.position + leg.zero[0],
                                              roll_it->second.position + leg.zero[1]);
        const Eigen::Vector2d motor_velocity(pitch_it->second.velocity, roll_it->second.velocity);
        if (!motor_position.allFinite() || !motor_velocity.allFinite())
        {
            publish_hold();
            return;
        }

        const auto solved = leg.solver.fk_pose(motor_position, leg.pose);
        if (!solved.ok())
        {
            publish_hold();
            return;
        }
        const Eigen::Vector2d ankle_velocity = leg.solver.fk_vel(motor_velocity, solved.position);
        if (!ankle_velocity.allFinite())
        {
            publish_hold();
            return;
        }

        leg.pose = solved.position;
        leg.pose_valid = true;
        pitch_it->second.position = static_cast<float>(solved.position[1]);
        roll_it->second.position = static_cast<float>(leg.roll_sign * solved.position[0]);
        pitch_it->second.velocity = static_cast<float>(ankle_velocity[1]);
        roll_it->second.velocity = static_cast<float>(leg.roll_sign * ankle_velocity[0]);
    }

    std::array<Leg, 2> legs_{};
};
