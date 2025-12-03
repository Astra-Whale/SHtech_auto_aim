//
// Created for communication module separation - FoxgloveServer_t
//

#include "foxglove_server.hpp"


namespace foxgloveSer
{
    FoxgloveServer_t::FoxgloveServer_t(pipeline::bridge::EntryStageToFoxgloveRobotBridge& robot_bridge,
                                      pipeline::bridge::EntryStageToFoxgloveAliveBridge& alive_bridge)
        : BasicTask(), robot_bridge_(robot_bridge), alive_bridge_(alive_bridge)
    {
        // 注册消息接收回调
        robot_bridge_.set_receiver([this](const pipeline::bridge::EntryStageToFoxgloveRobotMessage& msg) {
            this->handle_robot_state_message(msg);
        });

        alive_bridge_.set_receiver([this](const pipeline::bridge::EntryStageToFoxgloveAliveMessage& msg) {
            this->handle_alive_message(msg);
        });

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


        auto log_channel_result = foxglove::schemas::LogChannel::create("/alive");
        if (!log_channel_result.has_value())
        {
            std::cerr << "Failed to create scene channel: " << foxglove::strerror(log_channel_result.error()) << '\n';
            return;
        }
        log_channel = std::make_unique<foxglove::schemas::LogChannel>(std::move(log_channel_result.value()));

        auto tf_channel_result = foxglove::schemas::FrameTransformChannel::create("/tf");
        if (!tf_channel_result.has_value())
        {
            std::cerr << "Failed to create tf channel: " << foxglove::strerror(tf_channel_result.error()) << '\n';
            return; 
        }
        tf_channel = std::make_unique<foxglove::schemas::FrameTransformChannel>(std::move(tf_channel_result.value()));
    }

    FoxgloveServer_t::~FoxgloveServer_t()
    {
    }

    
    void FoxgloveServer_t::handle_robot_state_message(const pipeline::bridge::EntryStageToFoxgloveRobotMessage& msg)
    {
        const auto& enemy_robot_pose = msg.enemy_robot_state;
        foxglove::schemas::Pose pose;
        if(_debug)
        {
            LOGM_S("[foxglove_server] State: x %.2f y %.2f z %.2f", enemy_robot_pose(0,0), enemy_robot_pose(2,0), enemy_robot_pose(4,0));
        }

        pose.position = foxglove::schemas::Vector3(enemy_robot_pose(0,0), enemy_robot_pose(2,0), enemy_robot_pose(4,0));


        foxglove::schemas::FrameTransform tf;
        tf.parent_frame_id = "world";
        tf.child_frame_id = "child";

        tf.translation = foxglove::schemas::Vector3(0.0, 0.0, 0.0);

        // 单位旋转（四元数 w=1, x=y=z=0）
    
        tf.rotation = foxglove::schemas::Quaternion(0.0, 0.0, 0.0, 1.0);
    
    
        tf_channel->log(tf);




        foxglove::schemas::CubePrimitive cube;
        cube.pose = pose;
        cube.size = foxglove::schemas::Vector3(0.23, 0.02, 0.13); // 设置立方体大小
        cube.color = foxglove::schemas::Color(1.0, 0.0, 0.0, 0.8); // 设置立方体颜色为半透明红色


        foxglove::schemas::SceneEntity entity;
        entity.id = "enemy_robot";
        entity.cubes.push_back(cube);
        entity.frame_id = "world";
        foxglove::schemas::SceneUpdate update;
        update.entities.push_back(entity);
        scene_channel->log(update);
    }

    void FoxgloveServer_t::handle_alive_message(const pipeline::bridge::EntryStageToFoxgloveAliveMessage& msg)
    {
        // 记录服务器存活状态的逻辑
        foxglove::schemas::Log log_msg;
        log_msg.level = foxglove::schemas::Log::LogLevel::INFO;
        log_msg.message = "Foxglove server is alive";
        log_channel->log(log_msg);
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