#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "FSM/State_RaisingHand.h"
#include "FSM/State_CarryBox.h"
#include "AISignal.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";
    std::cout << "\n";
    
    // Module selection from command line: -m 1 or -m 2 (default: 1)
    int module_choice = vm["module"].as<int>();
    
    std::cout << "=== Module Selection ===\n";
    std::cout << "  1. RaisingHand AI (AI controlled raising hand)\n";
    std::cout << "  2. CarryBox (Walk 5s then raise hand)\n";
    
    if (module_choice == 2) {
        AISignal::getInstance().setModule(AIModule::CARRY_BOX);
        std::cout << "\n>>> Module 2: CarryBox selected\n";
        std::cout << "    Scenario: Walk forward 5s -> Carrybox\n\n";
    } else {
        AISignal::getInstance().setModule(AIModule::RAISING_HAND_AI);
        std::cout << "\n>>> Module 1: RaisingHand AI selected (default)\n";
        std::cout << "    Scenario: Follow human\n\n";
    }
    
    std::cout << "Tip: Use -m 2 to select CarryBox module\n\n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    // Start AI Signal receiver 
    AISignal::getInstance().start("tcp://localhost:5555");
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

