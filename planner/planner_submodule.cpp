//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "planner_submodule.hpp"

namespace plan
{

    PlannerSubModule::PlannerSubModule(pipeline::bridge::PlannerToSerialBridge &message_bridge) 
        : SubModule(SubModuleName::PLANNER), planner_bridge(message_bridge)
    {
        LOGM_S("[PlannerSubModule] construction completed");
    }

    bool PlannerSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const {
        return false;
    }

    command_array_t PlannerSubModule::generate_command_array(const RobotCommand& command) {
        constexpr std::chrono::microseconds plan_period{2000};
        command_array_t commands;
        for (size_t i = 0; i < CMDARRAYLENGTH; ++i) {
            commands[i] = RobotCommand{
                command.distance,
                command.yaw_angle+i*command.yaw_speed*float(plan_period.count())/1e6f,
                command.yaw_speed,
                command.pitch_angle+i*command.pitch_speed*float(plan_period.count())/1e6f,
                command.pitch_speed,
                command.target_id,
                command.shoot_mode
            };
        }
        return commands;
    }

    SubModuleResult PlannerSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                     const pipeline::BasicTask* parent)
    {
        pipeline::bridge::PlannerToSerialMessage msg{
            generate_command_array(data->robotcommand),
            data->attitude,
            plan_period
        };
        planner_bridge.send(msg);
        return SubModuleResult::SUCCESS;
    }


}