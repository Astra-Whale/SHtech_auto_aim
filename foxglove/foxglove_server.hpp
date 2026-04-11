//
// Created for communication module separation - FoxgloveServer_t
//

#ifndef FOXGLOVE_SERVER_H
#define FOXGLOVE_SERVER_H

#include "common.hpp"

#include <cstdint>
#include <string>
#include <memory>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

// Foxglove headers
#include <foxglove/server.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/schemas.hpp>

namespace foxgloveSer
{
    struct FoxgloveServerConfig : pipeline::ModuleConfig
    {
    };

    class FoxgloveServer_t : public pipeline::BasicTask
    {
    public:
        FoxgloveServer_t(const FoxgloveServerConfig& config,
                        pipeline::bridge::EntryStageToFoxgloveRobotBridge& robot_bridge,
                        pipeline::bridge::EntryStageToFoxgloveAliveBridge& alive_bridge);
        virtual ~FoxgloveServer_t();

        void operator()() override;
 
    private:
        FoxgloveServerConfig config_;
        // 消息桥接引用
        pipeline::bridge::EntryStageToFoxgloveRobotBridge& robot_bridge_;
        pipeline::bridge::EntryStageToFoxgloveAliveBridge& alive_bridge_;

        // 消息处理方法
        void handle_robot_state_message(const pipeline::bridge::EntryStageToFoxgloveRobotMessage& msg);
        void handle_alive_message(const pipeline::bridge::EntryStageToFoxgloveAliveMessage& msg);

        std::unique_ptr<foxglove::McapWriter> writer;
        std::unique_ptr<foxglove::WebSocketServer> server;
        std::unique_ptr<foxglove::schemas::LogChannel> log_channel;
        std::unique_ptr<foxglove::schemas::SceneUpdateChannel> scene_channel;
        std::unique_ptr<foxglove::schemas::FrameTransformChannel> tf_channel;
        std::mutex writer_mutex;
    };
}

#endif // FOXGLOVE_SERVER_H