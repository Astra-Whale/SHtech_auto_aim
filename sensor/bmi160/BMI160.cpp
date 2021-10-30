#include "BMI160.hpp"
using namespace std::chrono;
bool BMI160::init()
{
    int bmi_i2c_bus_id = 8, bmi_i2c_addr = 0x68;
    char bus_name[32];
    char buf[16] = {0x2c, 0x03, 0x2c, 0x00};
    if (snprintf(bus_name, sizeof(bus_name), "/dev/i2c-%u", bmi_i2c_bus_id) < 0)
    {
        LOGE_S("Format i2c bus name error!");
        LOGE_F("Format i2c bus name error!");
        return false;
    }
    if ((bus = i2c_open(bus_name)) == -1)
    {
        LOGE_S("Open i2c bus:%s error!", bus_name);
        LOGE_F("Open i2c bus:%s error!", bus_name);
        return false;
    }
    memset(&bmi_i2c, 0, sizeof(bmi_i2c));
    i2c_init_device(&bmi_i2c);
    bmi_i2c.bus = bus;
    bmi_i2c.addr = bmi_i2c_addr & 0x3ff;
    bmi_i2c.page_bytes = 8;
    bmi_i2c.iaddr_bytes = 1;

    i2c_ioctl_write(&bmi_i2c, 0x40, buf, 4);
    std::this_thread::sleep_for(milliseconds(10));
    buf[0] = 0x11;
    i2c_ioctl_write(&bmi_i2c, 0x7E, buf, 1);
    std::this_thread::sleep_for(milliseconds(10));

    buf[0] = 0x15;
    i2c_ioctl_write(&bmi_i2c, 0x7E, buf, 1);
    std::this_thread::sleep_for(milliseconds(10));

    buf[0] = 0x19;
    i2c_ioctl_write(&bmi_i2c, 0x7E, buf, 1);
    std::this_thread::sleep_for(milliseconds(10));

    buf[0] = 0x03;
    i2c_ioctl_write(&bmi_i2c, 0x7E, buf, 1);
    std::this_thread::sleep_for(milliseconds(1000));
    init_success = true;
    return true;
}
void BMI160::read_handler()
{
    char buf[12];
    float data[6];
    mMahony.begin(SAMPLERATE);
    int cnt = 0;
    float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    float accel_filter_3[3] = {0.0f, 0.0f, 0.0f};
    float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
    bool fliter_init = false;
    do
    {
        auto tik = high_resolution_clock::now();
        i2c_ioctl_read(&bmi_i2c, 0x0C, buf, 12);
        for (int i = 0; i < 3; i++)
        {
            uint16_t origin = buf[i << 1 | 1] << 8 | buf[i << 1];
            int16_t scaled = (origin > 0x7fff) ? -(0xffff - origin) : origin;
            data[i] = scaled * 2000.0f / 0x7fff;
            if (!fliter_init)
            {
                accel_filter_3[i] = data[i];
                fliter_init = true;
            }
            else
            {
                accel_fliter_1[i] = accel_fliter_2[i];
                accel_fliter_2[i] = accel_filter_3[i];
                data[i] = accel_filter_3[i] = accel_fliter_2[i] * fliter_num[0] + accel_fliter_1[i] * fliter_num[1] + data[i] * fliter_num[2];
            }
        }
        int16_t temp_origin;
        i2c_ioctl_read(&bmi_i2c, 0x20, &temp_origin, 2);
        float temperature = 23.0f + (float)temp_origin / (1 << 9);
        for (int i = 3; i < 6; i++)
        {
            uint16_t origin = buf[i << 1 | 1] << 8 | buf[i << 1];
            int16_t scaled = (origin > 0x7fff) ? -(0xffff - origin) : origin;
            data[i] = scaled * 2 * 9.8f / 0x7fff;
        }

        mMahony.updateIMU(data[0], data[1], data[2], data[3], data[4], data[5]);
        if (++cnt == 20)
        {
            LOGM_S("%3.2f C,%4.1f,%4.1f,%4.1f,%4.2f,%4.2f,%4.2f", temperature, data[0], data[1], data[2], data[3], data[4], data[5]);
            //LOGM_S("%5.1f,%5.1f,%5.1f", mMahony.getYaw(), mMahony.getPitch(), mMahony.getRoll());
            cnt = 0;
        }
        auto tok = high_resolution_clock::now();
        std::this_thread::sleep_for(duration_cast<microseconds>(microseconds(DURATION_US) - (tok - tik)));
    } while (mRun);
}