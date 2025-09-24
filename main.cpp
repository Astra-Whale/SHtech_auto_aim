#include "main.hpp"
#include "common/pipeline.hpp"
#include "sensor/sensor_submodule.hpp"
#include "detect/detect_submodule.hpp"
#include "predict/LinearPredictorSubModule.hpp"

bool enemy;
bool run = false;

int totalFrameCounter = 0;

// 使用通用的 CompositeTask 架构
pipeline::CompositeTask sensor_composite;
pipeline::CompositeTask detect_composite;
pipeline::CompositeTask predict_composite;

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

    // 创建传感器子模块
    auto sensor_module = std::make_unique<sensor::SensorSubModule>(
        info["source"], info["imu"], info["port"], info["flip"]);
    sensor_module->setdebug(display["sensor_debug"]);
    sensor_module->setshow(display["sensor_show"]);

    // 创建检测子模块
    auto detect_module = std::make_unique<detect::DetectSubModule>(info["model"]);
    detect_module->setdebug(display["detect_debug"]);
    detect_module->setshow(display["detect_show"]);

    // 创建预测子模块
    auto predict_module = std::make_unique<predict::LinearPredictorSubModule>(
        info["camera_para"], atoi(info["latency"].c_str()));
    predict_module->setdebug(display["predic_debug"]);
    predict_module->setshow(display["predic_show"]);

    // 注册并初始化各个子模块到对应的复合任务中
    sensor_composite.register_submodule(std::move(sensor_module));
    sensor_composite.init();
    
    detect_composite.register_submodule(std::move(detect_module));
    detect_composite.init();

    predict_composite.register_submodule(std::move(predict_module));
    predict_composite.init();

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
