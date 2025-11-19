//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#ifndef PREDICT_LINEAR_PREDICTOR_SUBMODULE_H
#define PREDICT_LINEAR_PREDICTOR_SUBMODULE_H

// modules
#include "common.hpp"
#include "tools.hpp"
#include "kalman.h"
#include "cboard.hpp"

// packages
#include <ctime>
#include <array>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    /**
     * @brief   线性预测子模块
     * @details 合并原有 PredictSubModule 和 LinearPredictor 的功能，
     *          既实现 SubModule 接口用于 pipeline，又包含完整的预测算法实现
     */
    class LinearPredictorSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] camera_param 相机参数文件路径
         * @param[in] latency 通信延迟（毫秒）
         */
        LinearPredictorSubModule(const std::string& camera_param,communicationBoard::Cboard_t& cboard, int latency = 20);
        virtual ~LinearPredictorSubModule() = default;

        bool should_skip(std::shared_ptr<ThreadDataPack> data) const override;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        communicationBoard::Cboard_t& cboard; /*!< 通讯板接口 */
        // 来自原 PredictSubModule 的成员
        PositionTransform position_transform;   /*!< 位置变换器 */
        double comm_latency;                    /*!< 通信延迟（秒） */
        
        // 来自原 LinearPredictor 的成员
        bool last_track{false};                /*!< 上一次是否有追踪目标 */
        Pos3D last_pw;                         /*!< 上一次世界坐标 */
        bbox_t last_bbox;                      /*!< 上一次预测框 */
        
        // 卡尔曼滤波器相关类型定义
        using _filter = Kalman<1, 2>;
        using Matx1 = _filter::Matrix_x1d;
        using Matxx = _filter::Matrix_xxd;
        using Matxz = _filter::Matrix_xzd;
        using Matz1 = _filter::Matrix_z1d;
        using Matzx = _filter::Matrix_zxd;
        using Matzz = _filter::Matrix_zzd;
        
        _filter filter_x;                      /*!< x轴卡尔曼滤波器 */
        _filter filter_y;                      /*!< y轴卡尔曼滤波器 */
        _filter filter_yaw;                    /*!< 装甲板yaw角度滤波器 */
        
        /**
         * @brief   初始化卡尔曼滤波器
         */
        void initFilters();
        
        /**
         * @brief   执行预测算法的核心逻辑
         * @param[in,out] data 数据包
         */
        void predict(std::shared_ptr<ThreadDataPack> data);
    };
}

#endif // PREDICT_LINEAR_PREDICTOR_SUBMODULE_H