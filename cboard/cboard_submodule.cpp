//
// Created for communication module separation - CboardSubModule
// Extracted communication functionality from UartIMU
//

#include "cboard_submodule.hpp"
#include <functional>
#include <chrono>

namespace cboard
{
    /**
     * @brief 控制输入的最大范围
     *
     * @param input 输入量
     * @param max 最大值（绝对值）
     * @return float 约化的输出值
     */
    float val_limit(float input, float max)
    {
        return input < max ? input > -max ? input : -max : max;
    }

    // AngleFilter 实现
    void AngleFilter::reset()
    {
        angle = 0.f;
        init = true;
    }

    float AngleFilter::update(float input)
    {
        if (init)
        {
            angle = input;
            init = false;
        }
        else
        {
            angle = angle * 0.9f + input * 0.1f;
        }
        return angle;
    }

    float AngleFilter::output()
    {
        return angle;
    }

    // CboardSubModule 实现
    CboardSubModule::CboardSubModule(const std::string& device_name) : SubModule()
    {
        LOGM_S("[cboard_submodule] constructing with device: %s", device_name.c_str());
        
        // 初始化IMU通讯
        imu = new UartIMU(device_name);
        if (imu == nullptr || !imu->init())
        {
            LOGE_S("[cboard_submodule] Failed to initialize IMU communication");
        }
        else
        {
            LOGM_S("[cboard_submodule] IMU communication initialized successfully");
            // 立即启动通讯
            start();
        }
    }

    CboardSubModule::~CboardSubModule()
    {
        if (imu)
        {
            imu->close();
            delete imu;
        }
    }

    void CboardSubModule::start()
    {
        if (imu != nullptr)
        {
            imu->start();
            LOGM_S("[cboard_submodule] Communication started");
        }
    }

    void CboardSubModule::close()
    {
        if (imu != nullptr)
        {
            imu->close();
            LOGM_S("[cboard_submodule] Communication closed");
        }
    }

    bool CboardSubModule::is_open() const
    {
        return (imu != nullptr) && imu->is_open();
    }

    bool CboardSubModule::process(std::shared_ptr<ThreadDataPack>& data, 
                                  pipeline::BasicTask* parent)
    {
        if (!is_open())
        {
            pitch_angle_filter.reset();
            if (_debug)
            {
                LOGW_S("[cboard_submodule] Communication not open, skipping processing");
            }
            return true; // 即使通讯未开启，也允许数据传递到下游
        }

        // 发送控制指令
        auto &send = data->robotcommand;
        auto &_attitude = data->attitude;
        
        imu->transmit_cmd(
            _attitude.yaw + val_limit(send.yaw_angle, 10),
            send.yaw_speed,
            pitch_angle_filter.update(_attitude.pitch + val_limit(send.pitch_angle, 10)),
            send.pitch_speed, 
            send.distance,
            static_cast<uint8_t>(send.shoot_mode == ShootMode::COMMON)
        );

        if (_debug)
        {
            LOGM_S("[cboard_submodule][transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                    send.pitch_angle, _attitude.pitch,
                    pitch_angle_filter.output(),
                    send.yaw_angle, _attitude.yaw,
                    _attitude.yaw + val_limit(send.yaw_angle, 10),
                    send.yaw_speed);
        }

        // 读取IMU数据并更新数据包
        imu->get_attitude(data->attitude);
        imu->get_robotstatus(data->robotstatus);

        return true; // 成功处理，传递到下游
    }
}