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
        // 缓存外部记录的本次entrystage开始时间，恢复包中存储的上次entrystage开始时间
        std::chrono::steady_clock::time_point entrystage_start_time_cache = data->submodule_timestamps[static_cast<uint8_t>(SubModuleName::ENTRYSTAGE)].first;
        data->submodule_timestamps[static_cast<uint8_t>(SubModuleName::ENTRYSTAGE)].first = data->pipeline_enter_time;

        // 计算总耗时
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - data->pipeline_enter_time).count();

        // 存储模块耗时和模块间耗时，格式：[模块0耗时, gap0, 模块1耗时, gap1, ...]
        std::array<long, SUBMODULE_COUNT * 2 - 1> timings;
        timings.fill(-1);

        for (size_t i = 0; i < SUBMODULE_COUNT; ++i) {
            auto& timestamps = data->submodule_timestamps[i];
            if (timestamps.first.time_since_epoch().count() > 0 && timestamps.second.time_since_epoch().count() > 0) {
                // 计算模块耗时
                timings[i * 2] = std::chrono::duration_cast<std::chrono::microseconds>(
                    timestamps.second - timestamps.first).count();
                
                // 计算模块间耗时（当前模块开始 - 上一个模块结束）
                if (i > 0) {
                    auto& prev_timestamps = data->submodule_timestamps[i - 1];
                    if (prev_timestamps.second.time_since_epoch().count() > 0) {
                        timings[i * 2 - 1] = std::chrono::duration_cast<std::chrono::microseconds>(
                            timestamps.first - prev_timestamps.second).count();
                    }
                }
            }
        }
        
        if(_debug)
        {
            LOGM_S("[EntryStageSubModule] recording frame index: %d", data->index);
            LOGM_S("[EntryStageSubModule] total time cost: %ld ms", total_duration);
            
            // 打印各模块耗时和模块间耗时
            for (size_t i = 0; i < SUBMODULE_COUNT; ++i) {
                if (timings[i * 2] >= 0) {
                    const char* module_name = getSubModuleName(static_cast<SubModuleName>(i));
                    if (i > 0 && timings[i * 2 - 1] >= 0) {
                        LOGM_S("[%s] %ldμs | gap: %ldμs", module_name, timings[i * 2], timings[i * 2 - 1]);
                    } else {
                        LOGM_S("[%s] %ldμs", module_name, timings[i * 2]);
                    }
                }
            }
        }
        
        //记录敌方机器人位置到 Foxglove
        foxglove_server.log_server_alive();
        foxglove_server.log_enemy_robot(data->target_state);






        // 为下一轮做初始化
        data->index = totalframecounter++;
        data->submodule_results.fill(SubModuleResult::NOTYET);
        // 恢复为本次entrystage开始时间，供下一轮使用
        data->pipeline_enter_time = entrystage_start_time_cache; 
        data->submodule_timestamps[static_cast<uint8_t>(SubModuleName::ENTRYSTAGE)].first = entrystage_start_time_cache;
        return SubModuleResult::SUCCESS;
    }
}