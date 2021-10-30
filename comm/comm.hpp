#ifndef _COMM_H
#define _COMM_H

#include "protocol.h"

#include "common.hpp"

#include <serial/serial.h>
#include <string>
#include <iostream>

#define DEFAULT_BAUDRATE 115200
#define DEFAULT_BUFF_LENGTH 1000

#define SOF 0xA5
#define GIMCtrl_CMD_ID 0x0501
typedef struct __attribute__((packed))
{
    float yaw;  //
    float pit;  //
    float dist; //
    uint8_t shoot;
} detection_t;

//up
#define CMD_MCU_DATA 0x1021
typedef struct __attribute__((packed))
{
    float curr_yaw;   //绝对量 yaw顺时针为正
    float curr_pitch; //pit水平为0 向上为负
    float curr_row;
} pc_mcu_data_t;

class Comm
{
public:
    Comm();
    bool open(const std::string &port = "/dev/ttyUSB0");
    bool transmit(float yaw, float pit, float dist);
    bool isOpen() const;
    bool receive();

    ~Comm();

private:
    serial::Serial ser;
    ProtocolConsumer rec;
    uint8_t send[DEFAULT_BUFF_LENGTH];
    uint8_t recv[DEFAULT_BUFF_LENGTH];
    bool state;
};

extern pc_mcu_data_t gim_state;

#endif
