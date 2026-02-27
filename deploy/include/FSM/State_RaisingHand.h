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
enum G1ArmJointIndex {
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,
    LeftWristYaw = 21,
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,
    RightWristYaw = 28
};


class State_RaisingHand : public State_RLBase
{
public:
    State_RaisingHand(int state, std::string state_string = "RaisingHand") 
    : State_RLBase(state, state_string)
    {
        auto cfg = param::config["FSM"]["RaisingHand"];
        if(cfg["raise_duration"].IsDefined()) {
            raise_duration_ = cfg["raise_duration"].as<float>();
        }
    }

    void enter()
    {
        // Gọi enter của State_RLBase để khởi động policy cho chân
        State_RLBase::enter();
        
        // Store initial arm positions
        for(int i = LeftShoulderPitch; i <= RightWristYaw; ++i) {
            q0_arm_[i - LeftShoulderPitch] = lowstate->msg_.motor_state()[i].q();
        }

        t0_arm_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        // Gọi run của State_RLBase để policy điều khiển chân
        State_RLBase::run();
        
        // Override điều khiển tay
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_arm_;
        float ratio = std::clamp(t / raise_duration_, 0.0f, 1.0f);

        // Target: Giơ tay PHẢI, tay trái giữ nguyên
        std::array<float, 14> q_target = {
            // Left arm (giữ nguyên)
            q0_arm_[0],   // LeftShoulderPitch
            q0_arm_[1],   // LeftShoulderRoll
            q0_arm_[2],   // LeftShoulderYaw
            q0_arm_[3],   // LeftElbow
            q0_arm_[4],   // LeftWristRoll
            q0_arm_[5],   // LeftWristPitch
            q0_arm_[6],   // LeftWristYaw
            
            // Right arm (giơ lên)
            -0.65f,        // RightShoulderPitch (raise up)
            -0.15f,       // RightShoulderRoll (slightly out)
            q0_arm_[9],   // RightShoulderYaw
            0.7f,         // RightElbow
            q0_arm_[11],  // RightWristRoll
            q0_arm_[12],  // RightWristPitch
            q0_arm_[13]   // RightWristYaw
        };

        // Interpolate và set arm positions
        for(int i = LeftShoulderPitch; i <= RightWristYaw; ++i)
        {
            int idx = i - LeftShoulderPitch;
            float q_des = q0_arm_[idx] + (q_target[idx] - q0_arm_[idx]) * ratio;
            lowcmd->msg_.motor_cmd()[i].q() = q_des;
            lowcmd->msg_.motor_cmd()[i].kp() = 40;
            lowcmd->msg_.motor_cmd()[i].kd() = 1;
        }
    }

    void exit()
    {
        // Gọi exit của State_RLBase để dừng policy thread
        State_RLBase::exit();
    }

private:
    double t0_arm_;
    float raise_duration_ = 2.0f;
    std::array<float, 14> q0_arm_;
};

REGISTER_FSM(State_RaisingHand)
