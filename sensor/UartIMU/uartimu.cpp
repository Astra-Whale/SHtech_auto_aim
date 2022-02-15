#include "uartimu.hpp"
using namespace std::chrono;
void UartIMU::read_handler()
{
    LOGM_S("[UART IMU] HI!");
    do
    {
        comm->receive();
        attitude_buf.yaw = gim_state.curr_yaw;
        attitude_buf.pitch = gim_state.curr_pitch;
        attitude_buf.roll = 0;
        robotstatus_buf.robot_speed_mps = gim_state.shoot_speed;
        switch (robot_state.robot_id <= 10)
        {
        case true:
        {
            robotstatus_buf.enemy_color = EnemyColor::BLUE;
            robotstatus_buf.enemy[0] = robot_state.blue_7_robot_HP;
            robotstatus_buf.enemy[1] = robot_state.blue_1_robot_HP;
            robotstatus_buf.enemy[2] = robot_state.blue_2_robot_HP;
            robotstatus_buf.enemy[3] = robot_state.blue_3_robot_HP;
            robotstatus_buf.enemy[4] = robot_state.blue_4_robot_HP;
            robotstatus_buf.enemy[5] = robot_state.blue_5_robot_HP;
            break;
        }
        case false:
        {
            robotstatus_buf.enemy_color = EnemyColor::RED;
            robotstatus_buf.enemy[0] = robot_state.red_7_robot_HP;
            robotstatus_buf.enemy[1] = robot_state.red_1_robot_HP;
            robotstatus_buf.enemy[2] = robot_state.red_2_robot_HP;
            robotstatus_buf.enemy[3] = robot_state.red_3_robot_HP;
            robotstatus_buf.enemy[4] = robot_state.red_4_robot_HP;
            robotstatus_buf.enemy[5] = robot_state.red_5_robot_HP;
        }
        }
        // LOGM_S("[UART IMU] :Yaw: %.2f, Pitch: %2.f", attitude_buf.yaw, attitude_buf.pitch);
    } while (mRun);
}
