#include "main.hpp"
#include "main_submodules.hpp"

bool enemy;
bool run = false;

int totalFrameCounter = 0;

// 使用新的 CompositeTask 架构
pipeline_test::SensorCompositeTask sensor_composite;
pipeline_test::DetectCompositeTask detect_composite;
pipeline_test::PredictCompositeTask predict_composite;

void stop(int signal)
{
    sensor_composite.stop();
    detect_composite.stop();
    predict_composite.stop();
    LOGM_S("Quit");
    if (
        !sensor_composite.isalive()
        && !detect_composite.isalive()
        && !predict_composite.isalive()
    )
        exit(0);
}

bool init(void)
{
    signal(SIGINT, stop);
    signal(SIGSEGV, stop);
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

    sensor_composite.init(info["source"], info["imu"], info["port"], info["flip"]);
    detect_composite.init(info["model"]);
    predict_composite.init(info["camera_para"], atoi(info["latency"].c_str()));
    sensor_composite.setdebug(display["sensor_debug"]);
    detect_composite.setdebug(display["detect_debug"]);
    predict_composite.setdebug(display["predic_debug"]);
    sensor_composite.setshow(display["sensor_show"]);
    detect_composite.setshow(display["detect_show"]);
    predict_composite.setshow(display["predic_show"]);

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
    autoaim_pipeline cap2det(2), det2pre(2), pre2cap(max_mem + 1);
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
                           { sensor_composite(pre2cap, cap2det); });

    t_detect = std::thread([&]()
                           { detect_composite(cap2det, det2pre); });

    t_predict = std::thread([&]()
                            { predict_composite(det2pre, pre2cap); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    t_predict.join();
    LOGM_S("Predict Thread Quit Success!");
    t_sensor.join();
    LOGM_S("Read Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    LOGM(screen, "Successfully Quit!");

    return 0;
}
