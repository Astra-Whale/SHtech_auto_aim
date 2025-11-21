//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "planner_submodule.hpp"

namespace plan
{

    PlannerSubModule::PlannerSubModule() : SubModule(SubModuleName::PLANNER)
    {
        LOGM_S("[PlannerSubModule] construction completed");
    }

    bool PlannerSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const {
        return false;
    }

    SubModuleResult PlannerSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                     const pipeline::BasicTask* parent)
    {
        return SubModuleResult::SUCCESS;
    }
}