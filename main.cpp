#include "main.hpp"

bool enemy;
bool run = false;

int totalFrameCounter = 0;

// 使用通用的 PipelineTask 架构
pipeline::PipelineTask* sensor_composite = nullptr;
pipeline::PipelineTask* detect_composite = nullptr;
pipeline::PipelineTask* predict_composite = nullptr;
communicationBoard::Cboard_t* cboard = nullptr;
foxgloveSer::FoxgloveServer_t* foxglove_server = nullptr;

void terminate(int signal)
{
    LOGM_S("Received termination signal, shutting down all tasks...");
    
    // 终止所有复合任务（这会唤醒等待的线程并让它们退出）
    if (cboard) cboard->terminate();
    if (sensor_composite) sensor_composite->terminate();
    if (detect_composite) detect_composite->terminate();
    if (predict_composite) predict_composite->terminate();
    if (foxglove_server) foxglove_server->terminate();
    
    LOGM_S("Quit");
    if ((cboard && !cboard->isterminated())
        ||(sensor_composite && !sensor_composite->isterminated())
        || (detect_composite && !detect_composite->isterminated())
        || (predict_composite && !predict_composite->isterminated())
        || (foxglove_server && !foxglove_server->isterminated())
    )
    {
        LOGM_S("Quit failed!");
        exit(0);
    }
}

bool init(void)
{
    signal(SIGINT, terminate);
    // signal(SIGSEGV, terminate);
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

    // 初始化复合任务
    sensor_composite = new pipeline::PipelineTask();
    detect_composite = new pipeline::PipelineTask();
    predict_composite = new pipeline::PipelineTask();

    // 初始化独立任务
    cboard = new communicationBoard::Cboard_t(info["port"]);
    foxglove_server = new foxgloveSer::FoxgloveServer_t();

    // 注册各个子模块
    bool entrystage_submodule_registered = false;
    bool sensor_submodule_registered = false;
    bool cboard_submodule_registered = false;
    bool detect_submodule_registered = false;
    bool predict_submodule_registered = false;
    bool planner_submodule_registered = false;
    bool foxglove_server_submodule_registered = false;

    cboard_submodule_registered = true; // cboard is instantiated separately
    foxglove_server_submodule_registered = true; // foxglove_server is instantiated separately


    entrystage_submodule_registered = sensor_composite->register_submodule_with_params<entrystage::EntryStageSubModule>(*foxglove_server);

    sensor_submodule_registered = sensor_composite->register_submodule_with_params<sensor::SensorSubModule>(
        info["source"], info["flip"], *cboard);

    detect_submodule_registered = detect_composite->register_submodule_with_params<detect::DetectSubModule>(info["model"]);

    predict_submodule_registered = predict_composite->register_submodule_with_params<predict::LinearPredictorSubModule>(
        info["camera_para"],*cboard, atoi(info["latency"].c_str()));

    planner_submodule_registered = predict_composite->register_submodule_with_params<plan::PlannerSubModule>();

    // 设置各个任务的调试和显示选项
    cboard->setdebug(display["cboard_debug"]);
    cboard->setshow(display["cboard_show"]);

    sensor_composite->setdebug(display["sensor_debug"]);
    sensor_composite->setshow(display["sensor_show"]);

    detect_composite->setdebug(display["detect_debug"]);
    detect_composite->setshow(display["detect_show"]);

    foxglove_server->setdebug(display["foxglove_server_debug"]);
    foxglove_server->setshow(display["foxglove_server_show"]);

    predict_composite->setdebug(display["predic_debug"]);
    predict_composite->setshow(display["predic_show"]);

    
    // 检查所有关键子模块是否注册成功
    if (!entrystage_submodule_registered 
        || !sensor_submodule_registered 
        || !cboard_submodule_registered 
        || !detect_submodule_registered 
        || !predict_submodule_registered 
        || !foxglove_server_submodule_registered
        || !planner_submodule_registered) {
        LOGE_S("[init] Critical modules unavailable, system cannot start");
        return false;
    }

    if (false) {
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
        delete sensor_composite;
        delete detect_composite;
        delete predict_composite;
        delete cboard;
        delete foxglove_server;
        return 0;
    }

    const int max_mem = 4;
    autoaim_pipeline cap2det(2), det2pre(2), pre2cap(max_mem + 1);
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(std::make_shared<ThreadDataPack>());
    }

    std::thread t_sensor, t_detect, t_predict, t_cboard, t_foxglove_server;

    sigset_t oldmask;
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

    // 创建线程但初始状态为等待
    t_sensor = std::thread([&]()
                           { (*sensor_composite)(pre2cap, cap2det); });

    t_detect = std::thread([&]()
                           { (*detect_composite)(cap2det, det2pre); });

    t_predict = std::thread([&]()
                            { (*predict_composite)(det2pre, pre2cap); });
                            
    t_cboard = std::thread([&]()
                          { (*cboard)(); });

    t_foxglove_server = std::thread([&]()
                          { (*foxglove_server)(); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    // 启动所有任务开始工作
    LOGM_S("Starting all composite tasks...");
    cboard->start();
    sensor_composite->start();
    detect_composite->start();
    predict_composite->start();
    foxglove_server->start();
    LOGM_S("All composite tasks started successfully!");


// std::this_thread::sleep_for(std::chrono::milliseconds(5000));
// terminate(0);

    // 主线程等待所有工作线程结束
    t_cboard.join();
    LOGM_S("Cboard Thread Quit Success!");

    t_sensor.join();
    LOGM_S("Sensor Thread Quit Success!");
    
    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    
    t_predict.join();
    LOGM_S("Predict Thread Quit Success!");

    t_foxglove_server.join();
    LOGM_S("Foxglove Server Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    LOGM(screen, "Successfully Quit!");

    // 手动释放内存
    delete sensor_composite;
    delete detect_composite;
    delete predict_composite;
    delete cboard;
    delete foxglove_server;

    return 0;
}