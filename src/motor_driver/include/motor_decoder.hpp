#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include "common.hpp"
#include "utils.hpp"

// 电机解码器基类 - 定义统一的解码/编码接口
class MotorDecoder
{
public:
    virtual ~MotorDecoder() = default;

    // 解码 CAN 数据到电机状态
    virtual bool decode(const uint8_t data[8], MotorState &state) = 0;

    // 编码电机指令到 CAN 数据
    virtual bool encode(const MotorCmd &cmd, uint8_t data[8]) = 0;

    // 检查是否为有效的响应帧
    virtual bool is_valid_response(const uint8_t data[8]) const = 0;

    // 获取电机型号名称
    virtual std::string get_model_name() const = 0;

    // 获取电机参数
    virtual const MotorParams& get_params() const = 0;
};

// EC-A8112-P1-18 电机解码器 - 支持问答模式返回报文类型 1
class ECA8112Decoder : public MotorDecoder
{
public:
    ECA8112Decoder()
    {
        // EC-A8112-P1-18 电机参数
        params_.p_min = -12.5f;    params_.p_max = 12.5f;
        params_.v_min = -18.0f;    params_.v_max = 18.0f;
        params_.kp_min = 0.0f;     params_.kp_max = 500.0f;
        params_.kd_min = 0.0f;     params_.kd_max = 5.0f;
        params_.t_min = -90.0f;    params_.t_max = 90.0f;
        params_.i_min = -30.0f;    params_.i_max = 30.0f;
    }
    ~ECA8112Decoder() override = default;

    // 解码 EC-A8112-P1-18 电机状态
    // 数据格式 (问答模式返回报文类型 1):
    // Byte 0: [7:5] 帧类型 (0x01), [4:0] 保留
    // Byte 1-2: 位置 (16 bit)
    // Byte 3-4: 速度 (12 bit, 高 8 位在 Byte 3, 低 4 位在 Byte 4 高半字节)
    // Byte 4-5: 电流 (12 bit, 高 4 位在 Byte 4 低半字节, 低 8 位在 Byte 5)
    // Byte 6: 温度 (8 bit)
    // Byte 7: 保留
    bool decode(const uint8_t data[8], MotorState &state) override
    {
        if (!is_valid_response(data)) {
            return false;
        }

        uint16_t pos_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
        state.position = uint_to_float(pos_uint, params_.p_min, params_.p_max, 16);

        uint16_t vel_uint = (static_cast<uint16_t>(data[3]) << 4) | ((data[4] & 0xF0) >> 4);
        state.velocity = uint_to_float(vel_uint, params_.v_min, params_.v_max, 12);

        uint16_t elec_uint = ((data[4] & 0x0F) << 8) | data[5];
        state.electric = uint_to_float(elec_uint, params_.i_min, params_.i_max, 12);

        uint16_t temp_uint = data[6] & 0xFF;
        state.temperature = (temp_uint - 50) / 2.0f;

        return true;
    }

    // 编码 EC-A8112-P1-18 电机指令
    // 数据格式 (问答模式指令):
    // Byte 0: [7:5] 模式 (0x00), [4:0] Kp 高 5 位
    // Byte 1: [7:1] Kp 低 7 位, [0] Kd 最高位
    // Byte 2: Kd 低 8 位
    // Byte 3-4: 位置 (16 bit)
    // Byte 5-6: 速度 (12 bit, 高 8 位在 Byte 5, 低 4 位在 Byte 6 高半字节)
    // Byte 6-7: 力矩 (12 bit, 高 4 位在 Byte 6 低半字节, 低 8 位在 Byte 7)
    bool encode(const MotorCmd &cmd, uint8_t data[8]) override
    {
        float kp = clamp(cmd.kp, params_.kp_min, params_.kp_max);
        float kd = clamp(cmd.kd, params_.kd_min, params_.kd_max);
        float pos = clamp(cmd.position, params_.p_min, params_.p_max);
        float vel = clamp(cmd.velocity, params_.v_min, params_.v_max);
        float torque = clamp(cmd.torque, params_.t_min, params_.t_max);

        uint16_t kp_uint = float_to_uint(kp, params_.kp_min, params_.kp_max, 12);
        uint16_t kd_uint = float_to_uint(kd, params_.kd_min, params_.kd_max, 9);
        uint16_t pos_uint = float_to_uint(pos, params_.p_min, params_.p_max, 16);
        uint16_t vel_uint = float_to_uint(vel, params_.v_min, params_.v_max, 12);
        uint16_t torque_uint = float_to_uint(torque, params_.t_min, params_.t_max, 12);

        data[0] = (0x00 << 5) | ((kp_uint >> 7) & 0x1F);
        data[1] = ((kp_uint & 0x7F) << 1) | ((kd_uint & 0x100) >> 8);
        data[2] = kd_uint & 0xFF;
        data[3] = pos_uint >> 8;
        data[4] = pos_uint & 0xFF;
        data[5] = vel_uint >> 4;
        data[6] = (vel_uint & 0x0F) << 4 | (torque_uint >> 8);
        data[7] = torque_uint & 0xFF;

        return true;
    }

    // 检查是否为 EC-A8112-P1-18 电机的有效响应帧 (帧类型 = 0x01)
    bool is_valid_response(const uint8_t data[8]) const override
    {
        uint8_t frame_type = data[0] >> 5;
        return (frame_type == 0x01);
    }

    std::string get_model_name() const override
    {
        return "EC-A8112-P1-18";
    }

    const MotorParams& get_params() const override
    {
        return params_;
    }

private:
    MotorParams params_;
};

// 解码器工厂 - 根据电机型号创建对应的解码器实例
class MotorDecoderFactory
{
public:
    static std::unique_ptr<MotorDecoder> create(const std::string &model_name)
    {
        if (model_name == "EC-A8112-P1-18" || model_name == "default") {
            return std::make_unique<ECA8112Decoder>();
        }
        return nullptr;
    }
};
