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
        // 计算总耗时
        // 使用 EntryStage 的开始时间作为流水线开始时间，Planning 的结束时间作为流水线结束时间
        auto start_time = data->submodule_timestamps[static_cast<uint8_t>(SubModuleName::ENTRYSTAGE)].first;
        auto end_time = data->submodule_timestamps[static_cast<uint8_t>(SubModuleName::COUNT)-1].second;
        
        long total_duration = -1;
        if (start_time.time_since_epoch().count() > 0 && end_time.time_since_epoch().count() > 0) {
             total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        }

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
            LOGM_F("[EntryStageSubModule] recording frame index: %d", data->index);
            LOGM_F("[EntryStageSubModule] total time cost: %ld ms", total_duration);
            LOGM_F("[EntryStageSubModule] start time: %ld",
                   std::chrono::duration_cast<std::chrono::milliseconds>(start_time.time_since_epoch()).count());
            
            // 打印各模块耗时和模块间耗时
            for (size_t i = 0; i < SUBMODULE_COUNT; ++i) {
                if (timings[i * 2] >= 0) {
                    const char* module_name = getSubModuleName(static_cast<SubModuleName>(i));
                    if (i > 0 && timings[i * 2 - 1] >= 0) {
                        LOGM_F("\"%s\" %ldμs | gap: %ldμs", module_name, timings[i * 2], timings[i * 2 - 1]);
                    } else {
                        LOGM_F("\"%s\" %ldμs", module_name, timings[i * 2]);
                    }
                }
            }
            // 如果耗时过低，稍作等待以免日志刷得太快，仅在调试模式下启用
            if(total_duration >= 0&&total_duration <= 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }
        
        //记录敌方机器人位置到 Foxglove
        foxglove_server.log_server_alive();
        foxglove_server.log_enemy_robot(data->target_state);






        // 为下一轮做初始化
        data->index = totalframecounter++;
        data->submodule_results.fill(SubModuleResult::NOTYET);
        
        return SubModuleResult::SUCCESS;
    }
}