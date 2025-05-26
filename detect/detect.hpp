//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Detect armor from opencv mat
//

#ifndef DETECT_DETECT_H
#define DETECT_DETECT_H

//submodules
#include "backend.hpp"

//modules
#include "common.hpp"
#include "armor_corner_optimizer.hpp"

//packages
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

class TRTModule;
namespace detect
{
    using pipeline::BasicTask;
    using pipeline::autoaim_pipeline;

    /**
     * @brief   装甲板检测类
     */
    class Detect : public pipeline::BasicTask
    {

    private:
        bool center = false;
        SafeUniquePtr<BackEnd> model; /*!< unique_ptr 智能指针 指向一个用于 推理的 BackEnd 对象 在 init 期间完成初始化 */
        ArmorCornerOptimizer corner_optimizer;
    public:
        /**
         * @brief   装甲板检测初始化
         * @details 创建 TRTModule 实例用于 TensorRT 推理
         * @param[in] OnnxFileName 用于推理的 Onnx 文件路径
         */
        void init(const std::string OnnxFileName);

        /**
         * @brief   装甲板检测线程主函数
         * @details 读取上一线程提交至缓存队列的报文指针 detection_obj_t*::obj
         *          调用 model 指向的 TensorRT 推理器对报文中的 cv::Mat::frame 变量进行推理
         *          将推理结果写入报文中的 std::vector<bbox_t>::bboxes 变量
         *          将报文指针提交给下一线程的缓存队列
         * @param[in] pipebefore 与装甲板检测的上一流程交互的线程间通信对象
         * @param[in] pipeafter  与装甲板检测的下一流程交互的线程间通信对象
         * @note    通过 stop() 控制启停
         *          必须先进行初始化
         */
        void operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter);

                /**
         * @brief 设置角点优化器的二值化阈值
         * @param[in] threshold 二值化阈值
         */
        void setOptimizerThreshold(int threshold) { corner_optimizer.setBinaryThreshold(threshold); }
        
        /**
         * @brief 设置角点优化器的灯条参数
         * @param[in] params 灯条参数
         */
        void setOptimizerLightParams(const LightParams& params) { 
            corner_optimizer.setLightParams(params); 

    };
};
}

#endif //DETECT_DETECT_H
