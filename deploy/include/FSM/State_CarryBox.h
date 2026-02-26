// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSM/State_RLBase.h"
#include "LinearInterpolator.h"

/**
 * G1 29dof Joint Index:
 * Legs (0-11):
 *   Left: 0-5 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
 *   Right: 6-11
 * Waist (12-14): yaw, pitch, roll
 * Left Arm (15-21): shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
 * Right Arm (22-28): shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
 */

// Arm joint indices for G1 29dof
namespace CarryBoxJoint {
    constexpr int LeftShoulderPitch = 15;
    constexpr int LeftShoulderRoll = 16;
    constexpr int LeftShoulderYaw = 17;
    constexpr int LeftElbow = 18;
    constexpr int LeftWristRoll = 19;
    constexpr int LeftWristPitch = 20;
    constexpr int LeftWristYaw = 21;
    constexpr int RightShoulderPitch = 22;
    constexpr int RightShoulderRoll = 23;
    constexpr int RightShoulderYaw = 24;
    constexpr int RightElbow = 25;
    constexpr int RightWristRoll = 26;
    constexpr int RightWristPitch = 27;
    constexpr int RightWristYaw = 28;
}

/**
 * State_CarryBox: Kế thừa từ State_RLBase
 * - Policy vẫn chạy để điều khiển chân giữ cân bằng
 * - Giơ CẢ HAI tay lên (như đang bê thùng)
 */
class State_CarryBox : public State_RLBase
{
public:
    State_CarryBox(int state, std::string state_string = "CarryBox") 
    : State_RLBase(state, state_string)
    {
        auto cfg = param::config["FSM"]["CarryBox"];
        if(cfg["raise_duration"].IsDefined()) {
            raise_duration_ = cfg["raise_duration"].as<float>();
        }
    }

    void enter()
    {
        State_RLBase::enter();
        
        // Lưu vị trí ban đầu của tay
        for(int i = CarryBoxJoint::LeftShoulderPitch; i <= CarryBoxJoint::RightWristYaw; ++i) {
            q0_arm_[i - CarryBoxJoint::LeftShoulderPitch] = lowstate->msg_.motor_state()[i].q();
        }

        t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        State_RLBase::run();
        
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_arm_;
        float ratio = std::clamp(t / raise_duration_, 0.0f, 1.0f);

        // Target: Giơ CẢ HAI tay lên
        std::array<float, 14> q_target = {
            // Left arm (giơ lên)
            -0.5f,        // LeftShoulderPitch (raise up)
            0.15f,        // LeftShoulderRoll (slightly out)
            q0_arm_[2],   // LeftShoulderYaw
            0.7f,         // LeftElbow
            q0_arm_[4],   // LeftWristRoll
            q0_arm_[5],   // LeftWristPitch
            q0_arm_[6],   // LeftWristYaw
            
            // Right arm (giơ lên)
            -0.5f,        // RightShoulderPitch (raise up)
            -0.15f,       // RightShoulderRoll (slightly out)
            q0_arm_[9],   // RightShoulderYaw
            0.7f,         // RightElbow
            q0_arm_[11],  // RightWristRoll
            q0_arm_[12],  // RightWristPitch
            q0_arm_[13]   // RightWristYaw
        };

        // Interpolate và set arm positions
        for(int i = CarryBoxJoint::LeftShoulderPitch; i <= CarryBoxJoint::RightWristYaw; ++i)
        {
            int idx = i - CarryBoxJoint::LeftShoulderPitch;
            float q_des = q0_arm_[idx] + (q_target[idx] - q0_arm_[idx]) * ratio;
            lowcmd->msg_.motor_cmd()[i].q() = q_des;
            lowcmd->msg_.motor_cmd()[i].kp() = 40;
            lowcmd->msg_.motor_cmd()[i].kd() = 1;
        }
    }

    void exit()
    {
        State_RLBase::exit();
    }

private:
    double t0_arm_;
    float raise_duration_ = 2.0f;
    std::array<float, 14> q0_arm_;
};

REGISTER_FSM(State_CarryBox)
