#include "uartimu.hpp"
#include "packet.hpp"
#include <functional>
#include <chrono>

using namespace std::chrono;
UartIMU::UartIMU(const std::string device_name): m_device_name(device_name), m_serial()
{
    m_serial.register_handler(CMD_MCU_DATA, std::bind(&UartIMU::on_receive_imu, this, std::placeholders::_1, std::placeholders::_2));
    m_serial.register_handler(CMD_ROBOT_DATA, std::bind(&UartIMU::on_receive_sts, this, std::placeholders::_1, std::placeholders::_2));
}

void UartIMU::on_receive_imu(drivers::packet_data_t * packet_ptr, drivers::packet_length_t len)
{
    if (len != sizeof(pc_mcu_data_t))
        LOGM_S("[UART][ERROR] invalid data length");
    pc_mcu_data_t* _tmp_ptr = (pc_mcu_data_t*) packet_ptr;
    m_attitude.yaw = _tmp_ptr->curr_yaw;
    m_attitude.pitch = _tmp_ptr->curr_pitch;
    m_robotstatus.robot_speed_mps = _tmp_ptr->shoot_speed;
}

void UartIMU::on_receive_sts(drivers::packet_data_t * packet_ptr, drivers::packet_length_t len)
{
    if (len != sizeof(robot_data_t))
        LOGM_S("[UART][ERROR] invalid data length");
    robot_data_t robot_state = *((robot_data_t*)packet_ptr);
    if (m_robotstatus.robot_speed_mps < 10.0f)
    {
        m_robotstatus.robot_speed_mps = 10.0f;
    }
    if (0 < robot_state.robot_id && robot_state.robot_id < 20)
    {
        m_robotstatus.enemy_color = EnemyColor::BLUE;
        m_robotstatus.enemy[0] = robot_state.blue_7_robot_HP;
        m_robotstatus.enemy[1] = robot_state.blue_1_robot_HP;
        m_robotstatus.enemy[2] = robot_state.blue_2_robot_HP;
        m_robotstatus.enemy[3] = robot_state.blue_3_robot_HP;
        m_robotstatus.enemy[4] = robot_state.blue_4_robot_HP;
        m_robotstatus.enemy[5] = robot_state.blue_5_robot_HP;
    }
    else if (robot_state.robot_id >= 100)
    {
        m_robotstatus.enemy_color = EnemyColor::RED;
        m_robotstatus.enemy[0] = robot_state.red_7_robot_HP;
        m_robotstatus.enemy[1] = robot_state.red_1_robot_HP;
        m_robotstatus.enemy[2] = robot_state.red_2_robot_HP;
        m_robotstatus.enemy[3] = robot_state.red_3_robot_HP;
        m_robotstatus.enemy[4] = robot_state.red_4_robot_HP;
        m_robotstatus.enemy[5] = robot_state.red_5_robot_HP;
    }
    else
    {
        m_robotstatus.enemy_color = EnemyColor::GRAY;
    }
}

void UartIMU::transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float dist, uint8_t shoot)
{
    advv_detection_t data_to_send;
    data_to_send.yaw = yaw;
    data_to_send.yaw_spd = yaw_spd;
    data_to_send.pit = pitch;
    data_to_send.pitch_spd = pitch_spd;
    data_to_send.dist = dist;
    data_to_send.shoot = shoot;

    m_serial.send(GIMAdvv_CMD_ID, (drivers::packet_data_t*)&data_to_send, sizeof(data_to_send));
    m_serial.send_break();
}
