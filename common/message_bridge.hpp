//
// Message PushBridge for decoupling inter-module communication
// Header-only design for minimal compilation overhead
//

#ifndef COMMON_MESSAGE_BRIDGE_HPP
#define COMMON_MESSAGE_BRIDGE_HPP

#include <functional>
#include <array>
#include "datatype.hpp"

namespace pipeline {
namespace bridge {

/**
 * @brief   通用消息桥接类模板
 * @details 提供解耦的流水线模块间通信机制，避免模块间直接依赖
 *          使用 std::function 实现类型安全的回调机制
 * @tparam  MessageType 消息类型
 */
template<typename MessageType>
class PushBridge {
public:
    using CallbackFunc = std::function<void(const MessageType&)>;
    
    PushBridge() = default;
    ~PushBridge() = default;
    
    // 禁用拷贝
    PushBridge(const PushBridge&) = delete;
    PushBridge& operator=(const PushBridge&) = delete;
    
    /**
     * @brief   设置消息接收回调（初始化时调用）
     * @param[in] callback 回调函数
     */
    void set_receiver(CallbackFunc callback) {
        callback_ = std::move(callback);
    }
    
    /**
     * @brief   发送消息（高频调用）
     * @param[in] msg 要发送的消息
     */
    void send(const MessageType& msg) const {
        if (callback_) {
            callback_(msg);
        }
    }
    
    /**
     * @brief   检查是否已设置接收器
     */
    bool has_receiver() const {
        return static_cast<bool>(callback_);
    }
    
private:
    CallbackFunc callback_;
};

template<typename MessageType>
class PullBridge {
public:
    using ProviderFunc = std::function<MessageType()>;

    PullBridge() = default;
    ~PullBridge() = default;

    PullBridge(const PullBridge&) = delete;
    PullBridge& operator=(const PullBridge&) = delete;

    /**
     * @brief   设置数据提供者（在初始化时调用）
     * @param[in] provider 一个无参函数，返回 MessageType
     */
    void set_provider(ProviderFunc provider) {
        provider_ = std::move(provider);
    }

    /**
     * @brief   拉取最新数据
     * @return  若已设置 provider，则返回其结果；否则返回 MessageType 默认值
     */
    MessageType get() const {
        if (provider_) {
            return provider_();
        }
        return MessageType{};
    }

    /**
     * @brief   检查是否已设置数据提供者
     */
    bool has_provider() const {
        return static_cast<bool>(provider_);
    }

private:
    ProviderFunc provider_;
};

/**
 * @brief   Planner 到串口控制器的命令消息
 */
struct PlannerToSerialMessage {
    std::array<RobotCommand, 10> command_array;
    Attitude attitude;
    std::chrono::microseconds plan_period;
};

/**
 * @brief   EntryStage 到 Foxglove 的机器人状态消息
 */
struct EntryStageToFoxgloveRobotMessage {
    Eigen::Matrix<double, 6, 1> enemy_robot_state;
};

/**
 * @brief   EntryStage 到 Foxglove 的存活信号消息（无数据）
 */
struct EntryStageToFoxgloveAliveMessage {
    // 空消息，仅用于触发存活信号
};

struct SensorFromSerialAttitudeMessage {
    Attitude attitude;
};

struct SensorFromSerialRobotStatusMessage {
    RobotStatus robotstatus;
};

/**
 * @brief   类型别名：Planner -> Hardware::TimedSerial 消息桥接
 */
using PlannerToSerialBridge = PushBridge<PlannerToSerialMessage>;

/**
 * @brief   类型别名：EntryStage -> Foxglove 机器人状态消息桥接
 */
using EntryStageToFoxgloveRobotBridge = PushBridge<EntryStageToFoxgloveRobotMessage>;

/**
 * @brief   类型别名：EntryStage -> Foxglove 存活信号消息桥接
 */
using EntryStageToFoxgloveAliveBridge = PushBridge<EntryStageToFoxgloveAliveMessage>;

using SensorFromSerialAttitudeBridge = PullBridge<SensorFromSerialAttitudeMessage>;

using SensorFromSerialRobotStatusBridge = PullBridge<SensorFromSerialRobotStatusMessage>;

} // namespace bridge
} // namespace pipeline

#endif // COMMON_MESSAGE_BRIDGE_HPP
