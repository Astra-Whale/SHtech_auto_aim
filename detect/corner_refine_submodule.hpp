//
// Created for pipeline refactor - CornerRefineSubModule
// Moves armor corner refinement out of DetectSubModule
//

#ifndef DETECT_CORNER_REFINE_SUBMODULE_H
#define DETECT_CORNER_REFINE_SUBMODULE_H

#include "armor_corner_optimizer.hpp"
#include "common.hpp"

namespace detect
{
    const int bin_threshold_for_blue = 70;
    const int bin_threshold_for_red = 60;

    struct CornerRefineConfig : pipeline::ModuleConfig
    {
        bool adjust_threshold = false;
    };

    class CornerRefineSubModule : public pipeline::SubModule
    {
    public:
        explicit CornerRefineSubModule(const CornerRefineConfig& config);
        virtual ~CornerRefineSubModule() = default;

        SubModuleResult process(std::shared_ptr<ThreadDataPack> data,
                                const pipeline::BasicTask* parent) override;

    private:
        CornerRefineConfig config_;
        ArmorCornerOptimizer corner_optimizer;
        int binary_thres = 94;
    };
}

#endif // DETECT_CORNER_REFINE_SUBMODULE_H
