//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "planner_submodule.hpp"

namespace plan
{

    PlannerSubModule::PlannerSubModule(communicationBoard::Cboard_t& cboard) : SubModule(SubModuleName::PLANNER), cboard(cboard)
    {
        LOGM_S("[PlannerSubModule] construction completed");
    }

    bool PlannerSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const {
        return false;
    }

    PlannerSubModule::command_array_t PlannerSubModule::generate_command_array(const RobotCommand& command) {
        command_array_t commands;
        for (size_t i = 0; i < CMDARRAYLENGTH; ++i) {
            commands[i] = RobotCommand{
                command.distance,
                command.yaw_angle+i*command.yaw_speed*float(ctl_period.count())/1e6f,
                command.yaw_speed,
                command.pitch_angle+i*command.pitch_speed*float(ctl_period.count())/1e6f,
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
        cboard.set_robotcommand(
            generate_command_array(data->robotcommand), 
            data->attitude
        );
        return SubModuleResult::SUCCESS;
    }


}