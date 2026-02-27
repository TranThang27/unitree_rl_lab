#include "FSM/State_RLBase.h"
#include "FSM/State_CarryBox.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "AISignal.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

// Fixed velocity command: 0.5 m/s forward for first 5s after entering Velocity mode
// change "velocity_commands" to "fixed_velocity_commands" in deploy.yaml to use this

REGISTER_OBSERVATION(fixed_velocity_commands)
{
    // episode_length được reset về 0 khi vào Velocity state (env->reset())
    // step_dt thường là 0.02s
    float elapsed = env->episode_length * env->step_dt;

    if (elapsed < 5.0f) 
    {
        // Trong 5 giây đầu từ khi vào Velocity: đi tới với vận tốc 0.5 m/s
        return std::vector<float>{0.5f, 0.0f, 0.0f};
    } 
    else 
    {
        // Sau 5 giây: đứng yên
        return std::vector<float>{0.0f, 0.0f, 0.0f};
    }
}

// Zero velocity command: stand still, used for RaisingHand state
// change "velocity_commands" to "zero_velocity_commands" in deploy.yaml to use this
REGISTER_OBSERVATION(zero_velocity_commands)
{
    return std::vector<float>{0.0f, 0.0f, 0.0f};
}

// AI-controlled velocity command: receive velocity from Python PD controller via ZMQ
// Supports both Module 1 (AI signal) and Module 2 (CarryBox - phase-based)
REGISTER_OBSERVATION(ai_velocity_commands)
{
    AISignal& ai = AISignal::getInstance();
    AIModule module = ai.getModule();
    
    if (module == AIModule::CARRY_BOX) {
        // Module 2: CarryBox - read velocity from State_CarryBox's static variables
        float vel_x = State_CarryBox::target_vel_x.load();
        float vel_yaw = State_CarryBox::target_vel_yaw.load();
        return std::vector<float>{vel_x, 0.0f, vel_yaw};
    }
    
    // Module 1: AI-controlled velocity from Python PD controller
    float vel_x = ai.getVelX();
    float vel_y = ai.getVelY();
    float vel_ang = ai.getVelAng();
    
    return std::vector<float>{vel_x, vel_y, vel_ang};
}



// CarryBox velocity command: Walk forward 0.3 m/s for 5s, then stop
// Used with Module 2 (CARRY_BOX)
REGISTER_OBSERVATION(carrybox_velocity_commands)
{
    float elapsed = env->episode_length * env->step_dt;
    
    // Walk forward for 5 seconds
    if (elapsed < 5.0f) 
    {       
        return std::vector<float>{0.2f, 0.0f, 0.0f};  // 0.3 m/s forward
    } 
    else 
    {
        // After 5s: stop (RaisingHand will be triggered by State_RLBase)
        return std::vector<float>{0.0f, 0.0f, 0.0f};
    }
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    // Auto transition based on selected module
    // Module 1: RaisingHand when AI signal
    // Module 2: CarryBox after 5s walking
    
    // Module 1: RaisingHand triggered by AI signal
    if(FSMStringMap.right.count("RaisingHand"))
    {
        int raisinghand_id = FSMStringMap.right.at("RaisingHand");
        this->registered_checks.emplace_back(
            std::make_pair(
                [this]() -> bool {
                    AISignal& ai = AISignal::getInstance();
                    if (ai.getModule() == AIModule::RAISING_HAND_AI) {
                        if (ai.isRaisingHandTriggered()) {
                            ai.clearRaisingHandTrigger();
                            spdlog::info("Module 1: RaisingHand triggered by AI signal");
                            return true;
                        }
                    }
                    return false;
                },
                raisinghand_id
            )
        );
    }
    
    // Module 2: CarryBox after 5s walking
    if(FSMStringMap.right.count("CarryBox"))
    {
        int carrybox_id = FSMStringMap.right.at("CarryBox");
        this->registered_checks.emplace_back(
            std::make_pair(
                [this]() -> bool {
                    AISignal& ai = AISignal::getInstance();
                    if (ai.getModule() == AIModule::CARRY_BOX) {
                        float elapsed = env->episode_length * env->step_dt;
                        if (elapsed >= 7.0f) {
                            return true;
                        }
                    }
                    return false;
                },
                carrybox_id
            )
        );
    }

    // Bad orientation check - transition to Passive if robot falls
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}