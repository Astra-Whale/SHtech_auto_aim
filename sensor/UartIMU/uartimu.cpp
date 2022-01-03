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
        // LOGM_S("[UART IMU] :Yaw: %.2f, Pitch: %2.f", attitude_buf.yaw, attitude_buf.pitch);
    } while (mRun);
}
