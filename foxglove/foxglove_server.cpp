//
// Created for communication module separation - FoxgloveServer_t
//

#include "foxglove_server.hpp"


namespace foxgloveSer
{
    FoxgloveServer_t::FoxgloveServer_t() : BasicTask()
    {

        // Start a server to communicate with the Foxglove app.
        foxglove::WebSocketServerOptions ws_options;
        ws_options.host = "127.0.0.1";
        ws_options.port = 8765;
        auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
        if (!server_result.has_value())
        {
            std::cerr << "Failed to create server: " << foxglove::strerror(server_result.error()) << '\n';
            return;
        }
        server = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value()));
        std::cerr << "Server listening on port " << server->port() << '\n';

        // Create a SceneUpdateChannel for logging changes to a 3d scene
        auto scene_channel_result = foxglove::schemas::SceneUpdateChannel::create("/scene");
        if (!scene_channel_result.has_value())
        {
            std::cerr << "Failed to create scene channel: " << foxglove::strerror(scene_channel_result.error()) << '\n';
            return;
        }
        scene_channel = std::make_unique<foxglove::schemas::SceneUpdateChannel>(std::move(scene_channel_result.value()));
    }

    FoxgloveServer_t::~FoxgloveServer_t()
    {
    }

    
    void FoxgloveServer_t::log_enemy_robot(Eigen::Matrix<double, 6, 1> enemy_robot_pose)
    {
        foxglove::schemas::CubePrimitive cube;
        cube.pose.position = foxglove::schemas::Vector3(enemy_robot_pose[0], enemy_robot_pose[2], enemy_robot_pose[4]);
        cube.size = foxglove::schemas::Vector3(0.23, 0.13, 0.1); // 设置立方体大小
        cube.color = foxglove::schemas::Color(1.0, 0.0, 0.0, 0.8); // 设置立方体颜色为半透明红色
        foxglove::schemas::SceneEntity entity;
        entity.id = "enemy_robot";
        entity.cubes.push_back(cube);
        foxglove::schemas::SceneUpdate update;
        update.entities.push_back(entity);
        scene_channel->log(update);
    }

    void FoxgloveServer_t::log_server_alive()
    {
        // 记录服务器存活状态的逻辑
        foxglove::schemas::Log msg;
        msg.level = foxglove::schemas::Log::LogLevel::INFO;
        msg.message = "Foxglove server is alive";
        channel->log(msg);
    }

    void FoxgloveServer_t::operator()()
    {
        while (true)
        {
            if (!wait_for_state_change())
            {
                break; // 收到终止信号，退出线程
            }

            while (isalive())
            {
                std::lock_guard<std::mutex> lock(writer_mutex);
                foxglove::McapWriterOptions mcap_options = {};
                
                // 确保mcap目录存在
                std::filesystem::path mcap_dir = "mcap";
                if (!std::filesystem::exists(mcap_dir))
                {
                    std::filesystem::create_directories(mcap_dir);
                }
                
                // 创建以时间戳命名的mcap文件
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                std::tm tm_now;
                localtime_r(&time_t_now, &tm_now);
                
                std::ostringstream filename;
                filename << "mcap/" << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".mcap";
                mcap_options.path = filename.str();
                
                auto writer_result = foxglove::McapWriter::create(mcap_options);
                if (!writer_result.has_value())
                {
                    std::cerr << "Failed to create writer: " << foxglove::strerror(writer_result.error()) << '\n';
                    return;
                }
                writer = std::make_unique<foxglove::McapWriter>(std::move(writer_result.value()));

                std::this_thread::sleep_for(std::chrono::seconds(60));

                if (writer)
                {
                    writer->close();
                    writer.reset();
                }
            }
            
        }
    }
}