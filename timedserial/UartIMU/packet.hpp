#ifndef TIMEDSERIAL_UARTIMU_PACKET_H
#define TIMEDSERIAL_UARTIMU_PACKET_H

/**
 * @brief 下行控制指令
 *
 * @note  包含角度信息和供预测的角速度信息
 */
#define GIMAdvv_CMD_ID 0x0503
typedef struct __attribute__((packed))
{
    float yaw;
    float pit;
    float yaw_spd;
    float pitch_spd;
    float yaw_acc;
    float pitch_acc;
    float dist;
    uint8_t shoot; // 0 is disable, 1 is enable continue shoot, 2 is self-determined, 3 is single shoot
    uint8_t target_id; // 1-7: robot, 8: outpost, 9: base, 0: none
} advv_detection_t;


/**
 * @brief 下行心跳包
 *
 * @note  自瞄状态信息 10Hz
 */
#define STS_CMD_ID 0x0500
typedef struct __attribute__((packed))
{
    uint8_t mode;
} detection_sts_t;

/**
 * @brief IMU位姿数据
 * 
 */
#define CMD_MCU_DATA 0x1021
typedef struct __attribute__((packed))
{
    float curr_yaw;   //绝对量 yaw顺时针为正
    float curr_pitch; // pit水平为0 向上为负
    float curr_roll;
    float shoot_speed;
    uint8_t autoaim_mode; // 1 if robot enter auto aim mode, 0 otherwise
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

#endif //TIMEDSERIAL_UARTIMU_PACKET_H