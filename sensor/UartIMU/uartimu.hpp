#ifndef SENSOR_IMU_UARTIMU_H
#define SENSOR_IMU_UARTIMU_H
// modules
#include "common.hpp"

// packages
#include <RMCVSerial/RMCVSerial.hpp>
#include <stdint.h>
#include <string>

class UartIMU
{
private:
    Attitude m_attitude;
    RobotStatus m_robotstatus;
    const std::string m_device_name;
    drivers::RMCVSerial m_serial;

public:
    UartIMU(const std::string device_name);
    bool init()
    {
        LOGM_S("Opening %s ", m_device_name.c_str());
        m_serial.open(m_device_name);
        return m_serial.is_open();
    }
    bool is_open() { return m_serial.is_open(); }
    void start()
    {
        m_serial.start_async_receive();
    }
    void close()
    {
        m_serial.stop_async_receive();
    }
    void on_receive_imu(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
    void on_receive_sts(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float dist, uint8_t shoot = 1, uint8_t target_id = 0);
    void get_attitude(Attitude &attitude)
    {
        attitude = m_attitude;
    }
    void get_quaternion(Eigen::Quaternionf &q)
    {
        m_attitude.toQuaternion(q);
    }
    void get_robotstatus(RobotStatus &robotstatus)
    {
        robotstatus = m_robotstatus;
    }
    ~UartIMU()
    {
        close();
    }

private:
    void read_handler();
};

#endif // SENOSR_IMU_UARTIMU_H
