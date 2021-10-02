#include "comm.h"
#include <algorithm>

pc_mcu_data_t gim_state = {0};

static void imu_handler(uint8_t *data, uint16_t length)
{
    gim_state = *(pc_mcu_data_t *)data;
    gim_state.curr_yaw = gim_state.curr_yaw / 180 * 3.14159f;
    gim_state.curr_pitch = gim_state.curr_pitch / 180 * 3.14159f;

    //update_pose(mcu_data->curr_yaw, mcu_data->curr_pitch, 0);
}

Comm::Comm() : state(false)
{
    rec.register_handler(CMD_MCU_DATA, imu_handler);
}

void Comm::open(const std::string &port)
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(DEFAULT_BAUDRATE);
        ser.setBytesize(serial::eightbits);
        ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_even);
        auto timeout = serial::Timeout(1000);
        ser.setTimeout(timeout);
        ser.open();
        std::cout << "Open Serial Success!" << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << "Open Serial Fail!" << std::endl;
    }
    state = true;
}

bool Comm::isOpen() const
{
    return state;
}

bool Comm::transmit(float x, float y, float z)
{
    if (!state)
    {
        return false;
    }

    uint16_t len;

    detection_t msg = {
        .yaw = x,
        .pit = y,
        .dist = z,
        .shoot = 1};
    len = protocol_provider.pack(send, SOF, GIMCtrl_CMD_ID, (uint8_t *)&msg, sizeof(detection_t));
    return ser.write(send, len);
}

bool Comm::receive()
{
    if (!state)
    {
        return false;
    }
    size_t length = ser.read(recv, std::min<size_t>(ser.available(), DEFAULT_BUFF_LENGTH));
    rec.unpack_process(recv, length);
    return true;
}

Comm::~Comm()
{
    return;
}