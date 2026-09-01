/**
 * @file main.cpp
 * @brief 程序入口和任务生命周期管理
 *
 * 当前流水线由 3 个 PipelineTask 和 1 个独立的 TimedSerial 任务组成：
 *
 * - stage 0：EntryStage、Sensor、Preprocess
 * - stage 1：Detect
 * - stage 2：CornerRefine、Predictor、Planner
 * - TimedSerial：通过消息桥接收规划指令，并向串口驱动发送控制量
 *
 * `init()` 负责创建任务、桥接和驱动。`main()` 创建线程并统一启动任务，收到
 * SIGINT 或 SIGTERM 后停止任务、等待线程退出，再释放任务和桥接对象。
 */


#include "main.hpp"


std::atomic<bool> g_stop_request{false};

pipeline::PipelineTask* pipeline_stage0 = nullptr;
pipeline::PipelineTask* pipeline_stage1 = nullptr;
pipeline::PipelineTask* pipeline_stage2 = nullptr;

hardware::TimedSerial* timed_serial = nullptr;
pipeline::bridge::PlannerToSerialBridge* planner_to_serial_bridge = nullptr;
pipeline::bridge::SensorFromSerialAttitudeBridge* sensor_from_serial_attitude_bridge = nullptr;
pipeline::bridge::SensorFromSerialRobotStatusBridge* sensor_from_serial_robot_status_bridge = nullptr;
// 专门处理 SIGSEGV 的函数
void segv_handler(int sig) {
    // 1. 输出一句简单的提示 (使用 write 是异步安全的，不要用 LOG/printf)
    const char* msg = "\n[CRITICAL] Segmentation Fault Detected!\n";
    write(STDERR_FILENO, msg, 40);

    // 2. 尝试打印堆栈信息
    void* array[20];
    int size = backtrace(array, 20);
    backtrace_symbols_fd(array, size, STDERR_FILENO);

    // 3. 直接暴力退出，不再尝试析构任何对象
    // 操作系统会负责回收内存和句柄
    _Exit(1); 
}

// 专门处理 SIGINT 的函数
void int_handler(int sig) {
    g_stop_request.store(true);
}

// 线程退出监控函数
struct ThreadMonitor {
    std::string name;
    std::future<void> join_task;
};

// 线程关闭等待超时时间
constexpr std::chrono::seconds THREAD_JOIN_TIMEOUT{3};

ThreadMonitor create_monitor(std::string name, std::thread& t) 
{
    return ThreadMonitor{
        name, 
        std::async(std::launch::async, [&t]() {
            if (t.joinable()) {
                t.join();
            }
        })
    };
};

bool init(void)
{
    signal(SIGINT, int_handler);
    signal(SIGTERM, int_handler);
    signal(SIGSEGV, segv_handler);
    screen = new Log();

    try {
    cmd_parser parser("launch.cfg");

    LOGM_F("open log file success!");
    LOGM_S("open log file success!");

    // 初始化 CoordTransformer 单例
    mathutils::CoordTransformer::Init(parser.get_string("camera_para"), parser.get_bool("transformer_adjust_armor_size"));
    LOGM_S("CoordTransformer initialized.");


    // 分配流水线任务
    pipeline_stage0 = new pipeline::PipelineTask();
    pipeline_stage1 = new pipeline::PipelineTask();
    pipeline_stage2 = new pipeline::PipelineTask();

    // 创建消息桥
    planner_to_serial_bridge = new pipeline::bridge::PlannerToSerialBridge();
    sensor_from_serial_attitude_bridge = new pipeline::bridge::SensorFromSerialAttitudeBridge();
    sensor_from_serial_robot_status_bridge = new pipeline::bridge::SensorFromSerialRobotStatusBridge();
    // 初始化任务并注入依赖

    hardware::TimedSerialConfig timed_serial_config;
    timed_serial_config.debug.log_text = parser.get_bool("timed_serial_log_text");

    entrystage::EntryStageConfig entrystage_config;
    entrystage_config.debug.log_file = parser.get_bool("entrystage_log_file");

    sensor::SensorConfig sensor_config;
    sensor_config.debug.log_text = parser.get_bool("sensor_log_text");
    sensor_config.debug.log_file = parser.get_bool("sensor_log_file");
    sensor_config.debug.show_image = parser.get_bool("sensor_show_image");

    detect::PreprocessConfig preprocess_config;
    preprocess_config.debug.log_text = parser.get_bool("preprocess_log_text");
    preprocess_config.debug.log_file = parser.get_bool("preprocess_log_file");
    preprocess_config.debug.show_image = parser.get_bool("preprocess_show_image");

    detect::DetectConfig detect_config;
    detect_config.debug.log_text = parser.get_bool("detect_log_text");
    detect_config.debug.log_file = parser.get_bool("detect_log_file");
    detect_config.debug.show_image = parser.get_bool("detect_show_image");

    detect::CornerRefineConfig corner_refine_config;
    corner_refine_config.debug.log_text = parser.get_bool("corner_refine_log_text");
    corner_refine_config.debug.log_file = parser.get_bool("corner_refine_log_file");
    corner_refine_config.debug.show_image = parser.get_bool("corner_refine_show_image");
    corner_refine_config.adjust_threshold = parser.get_bool("corner_refine_adjust_threshold");

    predict::MultiPolicyPredictorConfig predictor_config;
    predictor_config.debug.log_text = parser.get_bool("predictor_log_text");
    predictor_config.debug.log_file = parser.get_bool("predictor_log_file");
    predictor_config.debug.show_image = parser.get_bool("predictor_show_image");
    predictor_config.adjust_mode = parser.get_bool("predictor_adjust_mode");
    predictor_config.adjust_tracker_noise = parser.get_bool("predictor_adjust_tracker_noise");

    plan::PlannerConfig planner_config;
    planner_config.debug.log_text = parser.get_bool("planner_log_text");
    planner_config.debug.log_file = parser.get_bool("planner_log_file");
    planner_config.debug.show_image = parser.get_bool("planner_show_image");

    bool entrystage_submodule_registered = false;
    bool sensor_submodule_registered = false;
    bool preprocess_submodule_registered = false;
    bool detect_submodule_registered = false;
    bool corner_refine_submodule_registered = false;
    bool predict_submodule_registered = false;
    bool planner_submodule_registered = false;
    // 初始化独立任务
    std::unique_ptr<SerialInterface> driver;
    const std::string &port = parser.get_string("port");
    // 根据配置选择驱动类型
    if (port == "None" || port.empty()) {
        // 使用 MockDriver（无硬件模式）
        LOGW_S("[init] Using MockDriver (no hardware mode)");
        driver = std::make_unique<MockDriver>();
    } else {
        // 使用真实的 UartDriver
        LOGM_S("[init] Using UartDriver with port: %s", port.c_str());
        driver = std::make_unique<UartDriver>(port);
    }

    timed_serial = new hardware::TimedSerial(timed_serial_config,
                                            std::move(driver),
                                            *planner_to_serial_bridge,
                                            *sensor_from_serial_attitude_bridge,
                                            *sensor_from_serial_robot_status_bridge);

    entrystage_submodule_registered = pipeline_stage0->register_submodule_with_params<entrystage::EntryStageSubModule>(
        entrystage_config);

    sensor_submodule_registered = pipeline_stage0->register_submodule_with_params<sensor::SensorSubModule>(
        sensor_config,
        parser.get_string("source"),
        parser.get_string("flip"),
        *sensor_from_serial_attitude_bridge,
        *sensor_from_serial_robot_status_bridge);

    preprocess_submodule_registered = pipeline_stage0->register_submodule_with_params<detect::PreprocessSubModule>(preprocess_config);

    detect_submodule_registered = pipeline_stage1->register_submodule_with_params<detect::DetectSubModule>(detect_config, parser.get_string("model"));

    corner_refine_submodule_registered = pipeline_stage2->register_submodule_with_params<detect::CornerRefineSubModule>(corner_refine_config);

    predict_submodule_registered = pipeline_stage2->register_submodule_with_params<predict::MultiPolicyPredictorSubModule>(
        predictor_config);

    planner_submodule_registered = pipeline_stage2->register_submodule_with_params<plan::PlannerSubModule>(
        planner_config, *planner_to_serial_bridge, parser.get_string("planner_para"));

    // 集中注册 skip 依赖策略
    pipeline_stage0->register_skip_dependencies(SubModuleName::PREPROCESS,
                                                {SubModuleName::SENSOR});
    pipeline_stage1->register_skip_dependencies(SubModuleName::DETECT,
                                                {SubModuleName::SENSOR, SubModuleName::PREPROCESS});
    pipeline_stage2->register_skip_dependencies(SubModuleName::CORNER_REFINE,
                                                {SubModuleName::DETECT});
    pipeline_stage2->register_skip_dependencies(SubModuleName::MULTI_POLICY_PREDICTOR,
                                                {SubModuleName::SENSOR,
                                                 SubModuleName::PREPROCESS,
                                                 SubModuleName::DETECT,
                                                 SubModuleName::CORNER_REFINE});
    pipeline_stage2->register_skip_dependencies(SubModuleName::PLANNER,
                                                {SubModuleName::MULTI_POLICY_PREDICTOR});

    
    // 检查所有关键子模块是否注册成功
    if (!entrystage_submodule_registered 
        || !sensor_submodule_registered 
        || !preprocess_submodule_registered
        || !detect_submodule_registered 
        || !corner_refine_submodule_registered
        || !predict_submodule_registered 
        || !planner_submodule_registered
        ) 
    {
        LOGE_S("[init] Critical modules unavailable, system cannot start");
        return false;
    }

    LOGM_S("[init] all pipeline tasks registered successfully, system can start");
    return true;
    }
    catch (const std::exception &e)
    {
        LOGE_S("[init] Config/init failed: %s", e.what());
        return false;
    }
    catch (...)
    {
        LOGE_S("[init] Config/init failed: unknown exception");
        return false;
    }
}

int main(void)
{
    if (!init())
    {
        // 释放初始化阶段已分配的资源
        LOGE_S("Init Fail, Quit");
        delete pipeline_stage0;
        delete pipeline_stage1;
        delete pipeline_stage2;
        delete timed_serial;
        delete planner_to_serial_bridge;
        delete sensor_from_serial_attitude_bridge;
        delete sensor_from_serial_robot_status_bridge;
        return 0;
    }

    const int max_mem = 4;
    // cap2det 采用零缓冲握手机制，其他管道采用有缓冲队列
    pipeline::AutoAimHandshake cap2det(0);  // 零缓冲握手
    pipeline::AutoAimQueue det2pre(2);      // 有缓冲队列
    pipeline::AutoAimQueue pre2cap(max_mem + 1);  // 有缓冲队列
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(std::make_shared<ThreadDataPack>());
    }


    // 创建工作线程
    std::thread t_sensor, t_detect, t_predict, t_timed_serial;

    sigset_t oldmask;
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

    // 创建线程但初始状态为等待
    t_sensor = std::thread([&]()
                           { (*pipeline_stage0)(pre2cap, cap2det); });

    t_detect = std::thread([&]()
                           { (*pipeline_stage1)(cap2det, det2pre); });

    t_predict = std::thread([&]()
                            { (*pipeline_stage2)(det2pre, pre2cap); });
                            
    t_timed_serial = std::thread([&]()
                          { (*timed_serial)(); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    // 启动所有任务开始工作
    LOGM_S("Starting all tasks...");
    timed_serial->start();
    pipeline_stage0->start();
    pipeline_stage1->start();
    pipeline_stage2->start();
    LOGM_S("All tasks started successfully!");



    // 停止工作线程
    // 阻塞轮询等待终止信号
    while (!g_stop_request.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LOGM_S("Graceful shutdown initiated...");

    // 向各个线程发出终止信号
    if (timed_serial) timed_serial->terminate();
    if (pipeline_stage0) pipeline_stage0->terminate();
    if (pipeline_stage1) pipeline_stage1->terminate();
    if (pipeline_stage2) pipeline_stage2->terminate();

    // 将各个线程的 Join 任务放入监控列表
    std::vector<ThreadMonitor> monitors;

    monitors.push_back(create_monitor("TimedSerial", t_timed_serial));
    monitors.push_back(create_monitor("Sensor", t_sensor));
    monitors.push_back(create_monitor("Detect", t_detect));
    monitors.push_back(create_monitor("Predict", t_predict));

    // 主线程带超时地轮询所有任务
    auto deadline = std::chrono::steady_clock::now() + THREAD_JOIN_TIMEOUT;
    bool all_finished = false;

    while (std::chrono::steady_clock::now() < deadline) {
        all_finished = true;
        for (auto& mon : monitors) {
            // wait_for(0) 非阻塞检查状态
            if (mon.join_task.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                all_finished = false;
                break; // 只要有一个没完，就继续等
            }
        }
        
        if (all_finished) break; // 全部成功 Join，提前退出循环
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    // 检查是否有卡住的线程，如果有则打印日志并强制退出
    if (all_finished) {
        LOGM_S("All threads joined successfully!");
    } else {
        std::string stuck_threads = "";
        for (auto& mon : monitors) {
            if (mon.join_task.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                stuck_threads += "[" + mon.name + "] ";
                LOGE_S("TIMEOUT: Thread %s is STUCK and failed to join!", mon.name.c_str());
            } else {
                 LOGM_S("Thread %s joined normally.", mon.name.c_str());
            }
        }
        
        // 打印致命错误并强制退出
        const char* msg = "\n[FATAL] System Hang Detected! Force Quitting.\n";
        write(STDERR_FILENO, msg, strlen(msg));
        std::cerr << "Stuck Threads: " << stuck_threads << std::endl;
        
        // 直接自杀，不执行析构（防止死锁）
        _Exit(1); 
    } 
    LOGM(screen, "Successfully Quit!");
    // 释放资源
    delete pipeline_stage0;
    delete pipeline_stage1;
    delete pipeline_stage2;
    delete timed_serial;
    delete planner_to_serial_bridge;
    delete sensor_from_serial_attitude_bridge;
    delete sensor_from_serial_robot_status_bridge;
    
    // 销毁 CoordTransformer 单例
    mathutils::CoordTransformer::Destroy();
    return 0;
}
