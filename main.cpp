#include "main.hpp"

bool enemy;
bool run = false;

int totalFrameCounter = 0;

sensor::Sensor sensor_reader;
detect::Detect detector;
predict::Predict predictor;

void stop(int signal)
{
    sensor_reader.stop();
    detector.stop();
    predictor.stop();
    LOGM_S("Quit");
}

bool init(void)
{
    //signal(SIGINT, stop);
    //signal(SIGSEGV, stop);
    cmd_parser parser;
    map<string, string> info;
    map<string, bool> display;
    screen = new Log();
    try
    {
        parser.parse("launch.cfg", info, display);
    }
    catch (const char *msg)
    {
        LOGE(screen, "%s", msg);
        return false;
    }

    LOGM_F("open log file success!");
    LOGM_S("open log file success!");

    sensor_reader.init(info["source"], info["imu"], info["port"], info["flip"]);
    detector.init(info["model"]);
    predictor.init(info["camera_para"], atoi(info["latency"].c_str()));
    sensor_reader.setdebug(display["sensor_debug"]);
    detector.setdebug(display["detect_debug"]);
    predictor.setdebug(display["predic_debug"]);
    sensor_reader.setshow(display["sensor_show"]);
    detector.setshow(display["detect_show"]);
    predictor.setshow(display["predic_show"]);

    return true;
}

int main(void)
{

    if (!init())
    {
        LOGE_S("Init Fail, Quit");
        return 0;
    }

    const int max_mem = 5;
    autoaim_pipline cap2det(2), det2pre(2), pre2cap(max_mem + 1);
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(std::make_shared<ThreadDataPack>());
    }

    std::thread t_sensor, t_detect, t_predict;

    sigset_t oldmask;
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

    t_sensor = std::thread([&]()
                           { sensor_reader(pre2cap, cap2det); });

    t_detect = std::thread([&]()
                           { detector(cap2det, det2pre); });

    t_predict = std::thread([&]()
                            { predictor(det2pre, pre2cap); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    t_sensor.join();
    LOGM_S("Read Thread Quit Success!");
    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    t_predict.join();
    LOGM_S("Predict Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    LOGM(screen, "Successfully Quit!");

    return 0;
}
