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

void terminate(int signal)
{
    LOGM_S("Received termination signal, shutting down all tasks...");
    
    // 终止所有复合任务（这会唤醒等待的线程并让它们退出）
    sensor_composite.terminate();
    detect_composite.terminate();
    predict_composite.terminate();
    
    LOGM_S("All tasks termination signal sent");
    
    // 检查终止状态（可选的安全检查）
    if (!sensor_composite.isterminated() || 
        !detect_composite.isterminated() || 
        !predict_composite.isterminated())
    {
        LOGE_S("Warning: Some tasks may not have received termination signal properly");
        exit(1);
    }
}

bool init(void)
{
    signal(SIGINT, terminate);
    signal(SIGSEGV, terminate);
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

    // 注册复合任务（容错处理）
    bool sensor_registered = false;
    bool detect_registered = false;
    bool predict_registered = false;

    // 尝试注册传感器复合任务
    sensor_registered = sensor_composite.register_submodule_with_params<sensor::SensorSubModule>(
        info["source"], info["imu"], info["port"], info["flip"]);
    if (sensor_registered) {
        sensor_composite.setdebug(display["sensor_debug"]);
        sensor_composite.setshow(display["sensor_show"]);
        // 任务默认处于等待状态，无需显式调用stop()
    }

    // 尝试注册检测复合任务
    detect_registered = detect_composite.register_submodule_with_params<detect::DetectSubModule>(info["model"]);
    if (detect_registered) {
        detect_composite.setdebug(display["detect_debug"]);
        detect_composite.setshow(display["detect_show"]);
        // 任务默认处于等待状态，无需显式调用stop()
    }

    // 尝试注册预测复合任务
    predict_registered = predict_composite.register_submodule_with_params<predict::LinearPredictorSubModule>(
        info["camera_para"], atoi(info["latency"].c_str()));
    if (predict_registered) {
        predict_composite.setdebug(display["predic_debug"]);
        predict_composite.setshow(display["predic_show"]);
        // 任务默认处于等待状态，无需显式调用stop()
    }

    if (!sensor_registered || !detect_registered||!predict_registered) {
        LOGE_S("[init] Critical modules unavailable, system cannot start");
        return false;
    }

    if(false){
        LOGE_S("[init] some composite tasks unavailable, system still can start");
    }

    LOGE_S("[init] all composite tasks registered successfully, system can start");
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

    // 创建线程但初始状态为等待
    t_sensor = std::thread([&]()
                           { sensor_composite(pre2cap, cap2det); });

    t_detect = std::thread([&]()
                           { detect_composite(cap2det, det2pre); });

    t_predict = std::thread([&]()
                            { predict_composite(det2pre, pre2cap); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    // 启动所有任务开始工作
    LOGM_S("Starting all composite tasks...");
    sensor_composite.start();
    detect_composite.start();
    predict_composite.start();
    LOGM_S("All composite tasks started successfully!");

    // 主线程等待所有工作线程结束
    t_sensor.join();
    LOGM_S("Sensor Thread Quit Success!");
    
    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    
    t_predict.join();
    LOGM_S("Predict Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    LOGM(screen, "Successfully Quit!");

    return 0;
}
