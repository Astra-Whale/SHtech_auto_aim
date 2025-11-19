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
    class FoxgloveServer_t : public pipeline::BasicTask
    {
    public:
        FoxgloveServer_t();
        virtual ~FoxgloveServer_t();

        void operator()() override;

        void log_enemy_robot(Eigen::Matrix<double, 6, 1> enemy_robot_state);

        void log_server_alive();
 
    private:
        std::unique_ptr<foxglove::McapWriter> writer;
        std::unique_ptr<foxglove::WebSocketServer> server;
        std::unique_ptr<foxglove::schemas::LogChannel> channel;
        std::unique_ptr<foxglove::schemas::SceneUpdateChannel> scene_channel;
        std::mutex writer_mutex;
    };
}

#endif // FOXGLOVE_SERVER_H