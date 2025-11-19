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

// Foxglove headers
#include <foxglove/server.hpp>
#include <foxglove/schemas.hpp>

namespace foxgloveSer
{
    class FoxgloveServer_t : public pipeline::BasicTask
    {
    public:
        FoxgloveServer_t();
        virtual ~FoxgloveServer_t();

        void operator()() override;

    private:
        foxglove::WebSocketServerOptions ws_options;
        foxglove::WebSocketServer server;
        foxglove::schemas::LogChannel channel;
    };
}

#endif // FOXGLOVE_SERVER_H