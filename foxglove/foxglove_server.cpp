//
// Created for communication module separation - FoxgloveServer_t
//

#include "foxglove_server.hpp"

namespace foxgloveSer
{
    FoxgloveServer_t::FoxgloveServer_t() : BasicTask(), writer_(nullptr), channel_(nullptr)
    {
        LOGM_S("[foxglove_server_submodule] constructing");

        options.path = "log_example.mcap";
        options.truncate = true;

        auto writer_result = foxglove::McapWriter::create(options);
        if (!writer_result.has_value()) {
            throw std::runtime_error("Failed to create MCAP writer");
        }
        writer_ = std::make_unique<foxglove::McapWriter>(std::move(writer_result.value()));

        auto channel_result = foxglove::Channel::create("log", "json");
        if (!channel_result.has_value()) {
            throw std::runtime_error("Failed to create channel");
        }
        channel_ = std::make_unique<foxglove::Channel>(std::move(channel_result.value()));
    }


    FoxgloveServer_t::~FoxgloveServer_t()
    {
        if (writer_) { 
            writer_->close();
        }
        std::cout << "[foxglove_server_submodule] destroyed" << std::endl;
    }

    void FoxgloveServer_t::operator()()
    {
        // 统一的等待-工作循环
        while (true)
        {
            if (!wait_for_state_change())
            {
                break;  // 收到终止信号，退出线程
            }
            
            // 收到启动信号，开始工作循环
            while (isalive())
            {
                std::string message = R"({"timestamp": 123456789, "value": 1.0, "message": "Hello, Foxglove!"})";
                
                if (writer_ && channel_) {
                    writer_->log(*channel_, 
                                reinterpret_cast<const std::byte*>(message.data()), 
                                message.size());
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(5000)); 
            }
            
            // 工作循环结束（被stop），回到等待状态
        }
        
    }
}