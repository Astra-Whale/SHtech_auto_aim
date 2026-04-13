//
// Created for pipeline refactor - PreprocessSubModule
// Splits common detector image preprocessing out of DetectSubModule
//

#ifndef DETECT_PREPROCESS_SUBMODULE_H
#define DETECT_PREPROCESS_SUBMODULE_H

#include "common.hpp"

namespace detect
{
    struct PreprocessConfig : pipeline::ModuleConfig
    {
    };

    class PreprocessSubModule : public pipeline::SubModule
    {
    public:
        static constexpr int INPUT_W = 640;
        static constexpr int INPUT_H = 512;

        explicit PreprocessSubModule(const PreprocessConfig& config);
        virtual ~PreprocessSubModule() = default;

        SubModuleResult process(std::shared_ptr<ThreadDataPack> data,
                                const pipeline::BasicTask* parent) override;

    private:
        PreprocessConfig config_;
        cv::Mat resized_bgr_;
    };
}

#endif // DETECT_PREPROCESS_SUBMODULE_H
