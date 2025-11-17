//
// Created for communication module separation - FoxgloveServer_t
//

#ifndef FOXGLOVE_SERVER_H
#define FOXGLOVE_SERVER_H

// submodules

// modules
#include "common.hpp"

// packages
#include <stdint.h>
#include <string>
#include <functional>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <foxglove/channel.hpp>
#include <foxglove/mcap.hpp>

namespace foxgloveSer
{
    class FoxgloveServer_t : public pipeline::BasicTask
    {
    public:
        FoxgloveServer_t();
        virtual ~FoxgloveServer_t();

        void operator()() override;

    private:
        
        foxglove::McapWriterOptions options{};
        std::unique_ptr<foxglove::McapWriter> writer_;
        std::unique_ptr<foxglove::Channel> channel_;
    };
}

#endif // FOXGLOVE_SERVER_H