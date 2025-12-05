//
// Message PushBridge for decoupling inter-module communication
// Header-only design for minimal compilation overhead
//
/**
 * ======================================================================================
 * ⚠️ 安全性协议与架构契约 (SECURITY PROTOCOL & ARCHITECTURE CONTRACT) ⚠️
 * ======================================================================================
 * 本文件实现的 PushBridge/PullBridge 为了追求极致性能，使用了裸指针和 std::function。
 * 它本身 **不包含** 互斥锁保护。为了防止 内存崩溃(SegFault) 和 数据竞争(Data Race)，
 * 所有使用者必须严格遵守以下【全生命周期安全规范】：
 *
 * 1. 【初始化阶段 (Wiring Phase)】
 * - set_receiver / set_provider 必须且只能在 **单线程初始化阶段** 完成。
 * - 严禁在任何工作线程启动（std::thread 构造）之后修改回调函数。
 * - 此时 bridge 的回调被视为“只读”配置。
 *
 * 2. 【运行阶段 (Running Phase)】
 * - Sender 可以在任意线程并发调用 send()。
 * - Receiver 的回调函数必须自行保证线程安全（若访问共享资源）。
 * - 严禁在运行期间调用 set_receiver（会导致未定义的竞争行为）。
 *
 * 3. 【销毁阶段 (Destruction Phase)】
 * - 必须遵循“二段式关闭”原则：
 * Step A: 通知所有线程停止 (terminate flags)。
 * Step B: 主线程调用 join() 等待所有 Sender 线程彻底结束。
 * Step C: 只有在所有线程都 join 之后，才能 delete Bridge 和 Receiver 对象。
 * - 违反此顺序将导致 Use-After-Free (悬垂指针) 崩溃。
 *
 * ======================================================================================
 */

#ifndef COMMON_MESSAGE_BRIDGE_HPP
#define COMMON_MESSAGE_BRIDGE_HPP

#include "log/log.hpp"
#include <functional>
#include <array>
#include <chrono>
#include <Eigen/Dense>

#include "datatype.hpp"
#include "log/log.hpp"

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
        if (callback_) {
            LOGE_S("[Bridge] Receiver seted twice!");
            return;
        }
        callback_ = std::move(callback);
    }
    
    /**
     * @brief   发送消息（高频调用）
     * @param[in] msg 要发送的消息
     */
    void send(const MessageType& msg) const {
        try
        {
            if (callback_) {
                callback_(msg);
            }
        }
        catch(const std::exception& e)
        {
            LOGE_S("[Bridge] PushBridge send exception: %s", e.what());
        }
        catch (...)
        {
            LOGE_S("[Bridge] PushBridge send unknown exception");
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
        if (provider_) {
            LOGE_S("[Bridge] Provider seted twice!");
            return;
        }
        provider_ = std::move(provider);
    }

    /**
     * @brief   拉取最新数据
     * @return  若已设置 provider，则返回其结果；否则返回 MessageType 默认值
     */
    MessageType get() const {
        try
        {
            if (provider_) {
                return provider_();
            }
        }
        catch(const std::exception& e)
        {
            LOGE_S("[Bridge] PullBridge get exception: %s", e.what());
        }
        catch (...)
        {
            LOGE_S("[Bridge] PullBridge get unknown exception");
        }
        LOGE_S("[Bridge] No provider set for this PullBridge, returning default MessageType.");
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
