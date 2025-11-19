//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "entryStage_submodule.hpp"

namespace entrystage
{

    EntryStageSubModule::EntryStageSubModule(foxgloveSer::FoxgloveServer_t& foxglove_server) : SubModule(SubModuleName::ENTRYSTAGE), foxglove_server(foxglove_server)
    {
        LOGM_S("[EntryStageSubModule] construction completed");
    }

    bool EntryStageSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const {
        return false;
    }

    SubModuleResult EntryStageSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                     const pipeline::BasicTask* parent)
    {
        // 
        // 记录敌方机器人位置到 Foxglove
        foxglove_server.log_server_alive();
        foxglove_server.log_enemy_robot(data->enemy_robot_pose);






        // 为下一轮做初始化
        data->index = totalframecounter++;
        data->submodule_results.fill(SubModuleResult::NOTYET);
        return SubModuleResult::SUCCESS;
    }
}