#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <serial/serial.h>
#include "stdint.h"
#include <vector>
#include <map>

// Internal definitions
#ifdef _PROTOCOL_H

#define UP_REG_ID 0xA5
#define DN_REG_ID 0xA5
#define HEADER_LEN sizeof(frame_header_t)
#define CMD_LEN 2
#define CRC_LEN 2
#define PROTOCAL_FRAME_MAX_SIZE 200
#define UART_BUFF_SIZE 2000
#define PACK_MAX_SIZE 200
#define COMPRESS_TIME 1

typedef struct
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} __attribute__((packed)) frame_header_t;

#endif

class ProtocolConsumer
{
public:
    ProtocolConsumer();

    typedef void (*Handler)(uint8_t *data, uint16_t length);
    typedef void (*DefaultHandler)(uint16_t cmd_id, uint8_t *data, uint16_t length);

    void unpack_process(uint8_t *data, uint16_t length);

    Handler register_handler(uint16_t cmd_id, Handler Handler);
    Handler unregister_handler(uint16_t cmd_id);
    void set_default_handler(DefaultHandler Handler);

private:
    typedef enum
    {
        STEP_HEADER_SOF = 0,
        STEP_LENGTH_LOW = 1,
        STEP_LENGTH_HIGH = 2,
        STEP_FRAME_SEQ = 3,
        STEP_HEADER_CRC8 = 4,
        STEP_DATA_CRC16 = 5,
    } unpack_step_e;

    void handle_frame(frame_header_t *packet);

    uint8_t byte_, protocol_packet_[PROTOCAL_FRAME_MAX_SIZE];
    uint16_t data_length_;

    std::map<uint16_t, Handler> handlers;
    DefaultHandler default_handler = nullptr;

    uint32_t read_buff_index_, index_;

    unpack_step_e unpack_step;
};

class ProtocolProvider
{
public:
    uint16_t pack(uint8_t *buffer, uint8_t sof, uint16_t cmd_id, uint8_t *data, uint16_t length);
};

extern ProtocolProvider protocol_provider;

#endif