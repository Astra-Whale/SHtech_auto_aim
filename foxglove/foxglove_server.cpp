//
// Created for communication module separation - FoxgloveServer_t
//

#include "foxglove_server.hpp"

namespace foxgloveSer
{
    FoxgloveServer_t::FoxgloveServer_t() : BasicTask(),ws_options(),server(foxglove::WebSocketServer::create(std::move(ws_options)).value()),channel(foxglove::schemas::LogChannel::create("/hello").value())
    {

    }

    FoxgloveServer_t::~FoxgloveServer_t()
    {
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
              
                while (true)
                {
                  const auto now = std::chrono::system_clock::now();
                  const auto nanos_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                  const auto seconds_since_epoch = nanos_since_epoch / 1000000000;
                  const auto remaining_nanos = nanos_since_epoch % 1000000000;
              
                  foxglove::schemas::Log log;
                  foxglove::schemas::RawImage image;
                  log.level = foxglove::schemas::Log::LogLevel::INFO;
                  log.message = "Hello, Foxglove!";
                  log.timestamp = foxglove::schemas::Timestamp{
                      static_cast<uint32_t>(seconds_since_epoch),
                      static_cast<uint32_t>(remaining_nanos)};
                    log.
              
                  channel.log(log);
              
                  std::this_thread::sleep_for(std::chrono::milliseconds(333));
                }
                return;
            }
        }
    }
}