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

/*****************************************
 *               下行报文                 *
 *****************************************/

/**
 * @brief 基本控制指令
 *
 * @note  仅包含角度信息，不提供角速度信息
 */
#define GIMCtrl_CMD_ID 0x0501
typedef struct __attribute__((packed))
{
    float yaw;
    float pit;
    float dist;
    uint8_t shoot;
} detection_t;

/**
 * @brief 高级控制指令
 *
 * @note  包含角度信息和供预测的角速度信息
 */
#define GIMAdv_CMD_ID 0x0502
typedef struct __attribute__((packed))
{
    float yaw;
    float yaw_spd;
    float pit;
    float dist;
    uint8_t shoot;
} adv_detection_t;

/*****************************************
 *               上行报文                 *
 *****************************************/

/**
 * @brief IMU位姿数据
 * 
 */
#define CMD_MCU_DATA 0x1021
typedef struct __attribute__((packed))
{
    float curr_yaw;   //绝对量 yaw顺时针为正
    float curr_pitch; // pit水平为0 向上为负
    float shoot_speed;
} pc_mcu_data_t;

/**
 * @brief 赛场信息
 * 
 */
#define CMD_ROBOT_DATA 0x1022
typedef struct __attribute__((packed)) 
{ 
    uint16_t red_1_robot_HP;   
    uint16_t red_2_robot_HP;   
    uint16_t red_3_robot_HP;   
    uint16_t red_4_robot_HP;   
    uint16_t red_5_robot_HP;   
    uint16_t red_7_robot_HP;   
    uint16_t red_outpost_HP; 
    uint16_t red_base_HP;   
    uint16_t blue_1_robot_HP;   
    uint16_t blue_2_robot_HP;   
    uint16_t blue_3_robot_HP;   
    uint16_t blue_4_robot_HP;   
    uint16_t blue_5_robot_HP;   
    uint16_t blue_7_robot_HP;   
    uint16_t blue_outpost_HP; 
    uint16_t blue_base_HP;
    uint8_t robot_id;
} robot_data_t;

class Comm
{
public:
    Comm();
    bool open(const std::string &port = "/dev/ttyUSB0");
    bool transmit_Classic(float _yaw, float _pit, float _dist, uint8_t _shoot = 1);
    bool transmit(float _yaw, float _pit, float _spd, float _dist, uint8_t _shoot = 1);
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
extern robot_data_t robot_state;

#endif
