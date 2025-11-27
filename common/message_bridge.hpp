//
// Message Bridge for decoupling inter-module communication
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
class Bridge {
public:
    using CallbackFunc = std::function<void(const MessageType&)>;
    
    Bridge() = default;
    ~Bridge() = default;
    
    // 禁用拷贝
    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;
    
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

/**
 * @brief   Planner 到串口控制器的命令消息
 */
struct PlannerToSerialMessage {
    std::array<RobotCommand, 10> command_array;
    Attitude attitude;
};

/**
 * @brief   类型别名：Planner -> Hardware::TimedSerial 消息桥接
 */
using PlannerToSerialBridge = Bridge<PlannerToSerialMessage>;

} // namespace bridge
} // namespace pipeline

#endif // COMMON_MESSAGE_BRIDGE_HPP
