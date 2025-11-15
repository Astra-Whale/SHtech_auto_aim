//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "entryStage_submodule.hpp"

namespace entrystage
{

    EntryStageSubModule::EntryStageSubModule() : SubModule(SubModuleName::ENTRYSTAGE)
    {
        LOGM_S("[EntryStageSubModule] construction completed");
    }

    bool EntryStageSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const {
        return false;
    }

    SubModuleResult EntryStageSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                     const pipeline::BasicTask* parent)
    {
        // 入口阶段处理逻辑
        data->index = totalframecounter++;
        data->submodule_results.fill(SubModuleResult::NOTYET);
        return SubModuleResult::SUCCESS;
    }
}