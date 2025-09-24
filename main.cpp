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

    LOGM_S("[init] Starting submodule construction...");

    // 创建传感器子模块
    std::unique_ptr<sensor::SensorSubModule> sensor_module;
    try {
        sensor_module = std::make_unique<sensor::SensorSubModule>(
            info["source"], info["imu"], info["port"], info["flip"]);
        sensor_module->setdebug(display["sensor_debug"]);
        sensor_module->setshow(display["sensor_show"]);
        LOGM_S("[init] Sensor submodule created successfully");
    }
    catch (const std::exception& e) {
        LOGE_S("[init] Failed to create sensor submodule: %s", e.what());
        sensor_module = nullptr;
    }
    catch (...) {
        LOGE_S("[init] Failed to create sensor submodule: unknown error");
        sensor_module = nullptr;
    }

    // 创建检测子模块
    std::unique_ptr<detect::DetectSubModule> detect_module;
    try {
        detect_module = std::make_unique<detect::DetectSubModule>(info["model"]);
        detect_module->setdebug(display["detect_debug"]);
        detect_module->setshow(display["detect_show"]);
        LOGM_S("[init] Detect submodule created successfully");
    }
    catch (const std::exception& e) {
        LOGE_S("[init] Failed to create detect submodule: %s", e.what());
        detect_module = nullptr;
    }
    catch (...) {
        LOGE_S("[init] Failed to create detect submodule: unknown error");
        detect_module = nullptr;
    }

    // 创建预测子模块
    std::unique_ptr<predict::LinearPredictorSubModule> predict_module;
    try {
        predict_module = std::make_unique<predict::LinearPredictorSubModule>(
            info["camera_para"], atoi(info["latency"].c_str()));
        predict_module->setdebug(display["predic_debug"]);
        predict_module->setshow(display["predic_show"]);
        LOGM_S("[init] Predict submodule created successfully");
    }
    catch (const std::exception& e) {
        LOGE_S("[init] Failed to create predict submodule: %s", e.what());
        predict_module = nullptr;
    }
    catch (...) {
        LOGE_S("[init] Failed to create predict submodule: unknown error");
        predict_module = nullptr;
    }

    // 先组装复合任务（不管子模块是否成功）
    
    sensor_composite.register_submodule(std::move(sensor_module));
    detect_composite.register_submodule(std::move(detect_module));
    predict_composite.register_submodule(std::move(predict_module));

    // 验证复合任务状态
    auto sensor_status = sensor_composite.get_status();
    auto detect_status = detect_composite.get_status();
    auto predict_status = predict_composite.get_status();

    // 检查是否所有复合任务都准备就绪
    bool sensor_ready = sensor_composite.validate();
    bool detect_ready = detect_composite.validate();
    bool predict_ready = predict_composite.validate();

    if (!sensor_ready || !detect_ready || !predict_ready) {
        LOGE_S("[init] Some composite tasks are not ready:");
        LOGE_S("[init] - Sensor: %s", sensor_ready ? "READY" : "NOT READY");
        LOGE_S("[init] - Detect: %s", detect_ready ? "READY" : "NOT READY");
        LOGE_S("[init] - Predict: %s", predict_ready ? "READY" : "NOT READY");
        return false;
    }

    LOGM_S("[init] All composite tasks are ready and validated");
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
