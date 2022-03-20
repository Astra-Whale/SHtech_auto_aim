#include "comm.hpp"
#include <algorithm>

pc_mcu_data_t gim_state = {0};
robot_data_t robot_state = {0};

static void imu_handler(uint8_t *data, uint16_t length)
{
    gim_state = *(pc_mcu_data_t *)data;
    gim_state.curr_yaw = gim_state.curr_yaw;
    gim_state.curr_pitch = gim_state.curr_pitch;
}

static void robot_state_handler(uint8_t *data, uint16_t length)
{
    robot_state = *(robot_data_t *)data;
}

Comm::Comm() : state(false)
{
    rec.register_handler(CMD_MCU_DATA, imu_handler);
    rec.register_handler(CMD_ROBOT_DATA, robot_state_handler);
}

bool Comm::open(const std::string &port)
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(DEFAULT_BAUDRATE);
        ser.setBytesize(serial::eightbits);
        ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_even);
        auto timeout = serial::Timeout(0x7FFFFFFF, 1000, 0, 1000, 0);
        ser.setTimeout(timeout);
        ser.open();
        std::cout << "Open Serial Success!" << std::endl;
        state = true;
    }
    catch (std::exception &e)
    {
        std::cout << "Open Serial Fail!" << std::endl;
        state = false;
    }
    return state;
}

bool Comm::isOpen() const
{
    return state;
}

bool Comm::transmit_Classic(float _yaw, float _pit, float _dist, uint8_t _shoot)
{
    if (!state)
    {
        return false;
    }

    uint16_t len;

    detection_t msg = {
        .yaw = _yaw,
        .pit = _pit,
        .dist = _dist,
        .shoot = _shoot};
    len = protocol_provider.pack(send, SOF, GIMCtrl_CMD_ID, (uint8_t *)&msg, sizeof(detection_t));
    return ser.write(send, len);
}

bool Comm::transmit(float _yaw, float _pit, float _spd, float _dist, uint8_t _shoot)
{
    if (!state)
    {
        return false;
    }

    uint16_t len;

    adv_detection_t msg = {
        .yaw = _yaw,
        .yaw_spd = _spd,
        .pit = _pit,
        .dist = _dist,
        .shoot = _shoot
    };
    len = protocol_provider.pack(send, SOF, GIMAdv_CMD_ID, (uint8_t *)&msg, sizeof(adv_detection_t));
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
