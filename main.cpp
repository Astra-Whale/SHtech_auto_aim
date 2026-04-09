/**
 * ======================================================================================
 * 系统主入口与生命周期管理 (SYSTEM LIFECYCLE & DEV GUIDE)
 * ======================================================================================
 * 本文件负责管理整个机器人系统的 启动、连线、运行、停止 和 销毁。
 * 为了确保跨线程通信的内存安全，程序严格遵循以下顺序执行：
 *
 * [生命周期时序图]
 * 1. 【资源分配 (Allocation)】
 * - new 所有的 Bridge (消息桥)。
 * - new 所有的 Task (任务) 和 Driver (驱动)。
 *
 * 2. 【静态连线 (Wiring / Init)】
 * - 将 Bridge 传入各个 Task/Module 的构造函数。
 * - 在各模块构造函数内完成 set_receiver (绑定回调)。
 * - 此时严禁启动任何线程！
 *
 * 3. 【线程创建 (Thread Creation)】
 * - 创建 std::thread，线程进入等待或阻塞状态。
 * - 此时所有的 Bridge 连线必须已由主线程写入内存并对新线程可见。
 *
 * 4. 【系统启动 (Start)】
 * - 统一调用各 Task 的 start()，标志位翻转，工作循环正式开始。
 *
 * 5. 【运行监控 (Runtime)】
 * - 主线程阻塞在 while，等待信号函数改变原子变量 (SIGINT/Ctrl+C)。
 *
 * 6. 【安全停机 (Safe Shutdown)】 <--- 关键安全步骤
 * - 收到信号，调用 terminate() 设置退出标志。
 * - 异步执行 join()，检查各线程是否成功退出，带超时保护。
 * - 如果所有子线程完全退出主循环。则安全停机成功。此时不再有任何组件会访问 Bridge。
 * - 如果有线程卡死或者有段错误发生，打印错误日志并强制退出 (避免死锁)。
 *
 * 7. 【资源释放 (Cleanup)】
 * - delete 所有对象。由于线程已死，此时销毁是安全的。
 *
 * ======================================================================================
 * 开发人员自查清单 (DEV CHECKLIST) - 添加新任务/新通信时必读
 * ======================================================================================
 * 本系统包含四种核心对象：BasicTask (独立任务), PipelineTask (流水线任务), 
 * SubModule (流水线子模块), Bridge (消息桥)。
 * 
 * 
 * 
 * * --- [1] 对于 BasicTask (独立任务) ---
 * [ ] 1. 声明指针：
 * 在文件头部全局变量区声明 `Task* my_task = nullptr;`
 * [ ] 2. 分配内存 (Init):
 * 见下一步骤。
 * [ ] 3. 注入依赖 (Wiring):
 * 初始化标志位。
 * 在 init() 的 try-catch 块中 `new` 出对象。在构造函数中传入所需的 Bridge 和参数。
 * 统一检查启动成功与否。
 * [ ] 4. 线程管理 (Main):
 * (a) 声明线程：`std::thread t_my_task;`
 * (b) 绑定执行：`t_my_task = std::thread([&]{ (*my_task)(); });`
 * (c) 统一启动：`my_task->start();` (在所有线程创建后)
 * [ ] 5. 注册关闭 (Terminate):
 * (a) 添加停止信号 `if (my_task) my_task->terminate();`
 * (b) 将线程停止监控推入监控列表 `monitors.push_back(create_monitor("MyTask", t_my_task));`
 * [ ] 6. 内存释放 (Delete):
 * 在 main() 底部及 init() 异常处理块添加 `delete my_task;`
 * 
 * 
 * 
 * * --- [2] 对于 PipelineTask (流水线容器) ---
 * [ ] 1. 声明指针：
 * 在文件头部全局变量区声明 `pipeline::PipelineTask* my_composite = nullptr;`
 * [ ] 2. 分配内存 (Init):
 * 在 init() 开头 `new` 出对象。
 * [ ] 3. 注入依赖 (Wiring):
 * 通过 `register_submodule_with_params` 被子模块注入。
 * [ ] 4. 线程管理 (Main):
 * 同 BasicTask。注意 PipelineTask 的执行函数通常需要传入流水级间管道。
 * [ ] 5. 注册关闭 (Terminate):
 * 同 BasicTask。
 * [ ] 6. 内存释放 (Delete):
 * 同 BasicTask。
 * 
 * 
 * 
 * * --- [3] 对于 SubModule (流水线子模块) ---
 * [ ] 1. 声明指针：
 * 无。指针由 PipelineTask 内部管理。
 * [ ] 2. 分配内存 (Init):
 * 无。由 `register_submodule_with_params` 内部自动分配。
 * [ ] 3. 注入依赖 (Wiring):
 * 初始化标志位
 * 在 `register_submodule_with_params<Type>(args...)` 参数中传入 Bridge。
 * 在 init() 结尾统一检查注册返回值。
 * [ ] 4. 线程管理 (Main):
 * 无。运行在 PipelineTask 的线程中。
 * [ ] 5. 注册关闭 (Terminate):
 * 无。生命周期随 PipelineTask 自动管理。
 * [ ] 6. 内存释放 (Delete):
 * 无。随 PipelineTask 自动析构。
 * 
 * 
 * 
 * * --- [4] 对于 Bridge (PushBridge / PullBridge) ---
 * [ ] 1. 声明指针：
 * 在文件头部全局变量区声明 `Bridge* my_bridge = nullptr;`
 * [ ] 2. 分配内存 (Init):
 * 在 init() **最开始** `new` 出对象 (必须在 Task 之前)。
 * [ ] 3. 注入依赖 (Wiring):
 * 将 Bridge 引用传递给 Sender (Task) 和 Receiver (Task/SubModule)。
 * 必须确保 Receiver 在构造时完成了 `set_receiver`。
 * [ ] 4. 线程管理 (Main):
 * 无。
 * [ ] 5. 注册关闭 (Terminate):
 * 无。
 * [ ] 6. 内存释放 (Delete):
 * 在 main() 底部及 init() 异常处理块添加 `delete my_bridge;`
 * 注意：Bridge 必须在所有 Task 被 delete 后 (或至少 join 后) 才能 delete。
 *
 * ======================================================================================
 */


#include "main.hpp"


int totalFrameCounter = 0;

std::atomic<bool> g_stop_request{false};

// 第一步：声明指针
// 使用通用的 PipelineTask 架构
pipeline::PipelineTask* sensor_composite = nullptr;
pipeline::PipelineTask* detect_composite = nullptr;
pipeline::PipelineTask* predict_composite = nullptr;

hardware::TimedSerial* timed_serial = nullptr;
// foxgloveSer::FoxgloveServer_t* foxglove_server = nullptr;
pipeline::bridge::PlannerToSerialBridge* planner_to_serial_bridge = nullptr;

pipeline::bridge::EntryStageToFoxgloveRobotBridge* entrystage_to_foxglove_robot_bridge = nullptr;
pipeline::bridge::EntryStageToFoxgloveAliveBridge* entrystage_to_foxglove_alive_bridge = nullptr;
pipeline::bridge::SensorFromSerialAttitudeBridge* sensor_from_serial_attitude_bridge = nullptr;
pipeline::bridge::SensorFromSerialRobotStatusBridge* sensor_from_serial_robot_status_bridge = nullptr;
// 第一步结束


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
    std::thread* t_ptr;
    std::future<void> join_task;
};

// 线程关闭等待超时时间
constexpr std::chrono::seconds THREAD_JOIN_TIMEOUT{3};

ThreadMonitor create_monitor(std::string name, std::thread& t) 
{
    return ThreadMonitor{
        name, 
        &t,
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

    // 初始化 CoordTransformer 单例
    try {
        mathutils::CoordTransformer::Init(info["camera_para"], display["transformer_adjust"]);
        LOGM_S("CoordTransformer initialized.");
    } catch (const std::exception& e) {
        LOGE_S("[init] Failed to initialize CoordTransformer: %s", e.what());
        return false;
    }


    // 第二步: 分配内存
    // 初始化复合任务
    sensor_composite = new pipeline::PipelineTask();
    detect_composite = new pipeline::PipelineTask();
    predict_composite = new pipeline::PipelineTask();

    // 创建消息桥
    planner_to_serial_bridge = new pipeline::bridge::PlannerToSerialBridge();
    entrystage_to_foxglove_robot_bridge = new pipeline::bridge::EntryStageToFoxgloveRobotBridge();
    entrystage_to_foxglove_alive_bridge = new pipeline::bridge::EntryStageToFoxgloveAliveBridge();
    sensor_from_serial_attitude_bridge = new pipeline::bridge::SensorFromSerialAttitudeBridge();
    sensor_from_serial_robot_status_bridge = new pipeline::bridge::SensorFromSerialRobotStatusBridge();
    // 第二步结束


    // 第三步：初始化任务和注入依赖
    bool entrystage_submodule_registered = false;
    bool sensor_submodule_registered = false;
    bool preprocess_submodule_registered = false;
    bool detect_submodule_registered = false;
    bool corner_refine_submodule_registered = false;
    bool predict_submodule_registered = false;
    bool planner_submodule_registered = false;
    bool timed_serial_independenttask_registered = false;
    // bool foxglove_server_independenttask_registered = false;
    // 初始化独立任务
    try
    {
        std::unique_ptr<SerialInterface> driver;
        // 根据配置选择驱动类型
        if (info["port"] == "None" || info["port"].empty()) {
            // 使用 MockDriver（无硬件模式）
            LOGW_S("[init] Using MockDriver (no hardware mode)");
            driver = std::make_unique<MockDriver>();
        } else {
            // 使用真实的 UartDriver
            LOGM_S("[init] Using UartDriver with port: %s", info["port"].c_str());
            driver = std::make_unique<UartDriver>(info["port"]);
        }


        timed_serial = new hardware::TimedSerial(std::move(driver),
                                                *planner_to_serial_bridge,
                                                *sensor_from_serial_attitude_bridge,
                                                *sensor_from_serial_robot_status_bridge);
        timed_serial_independenttask_registered = true;
    }
    catch (const std::exception &e)
    {
        LOGE_S("[init] Failed to create TimedSerial: %s", e.what());
        timed_serial_independenttask_registered = false;
    }
    
    // try{
    //     foxglove_server = new foxgloveSer::FoxgloveServer_t(*entrystage_to_foxglove_robot_bridge, 
    //                                                     *entrystage_to_foxglove_alive_bridge);
    //     foxglove_server_independenttask_registered = true;
    // }
    // catch (const std::exception &e)
    // {
    //     LOGE_S("[init] Failed to create FoxgloveServer: %s", e.what());
    //     foxglove_server_independenttask_registered = false;
    // }

    // 将参数和桥传递给流水线子模块，并注册它们到流水级任务
    entrystage_submodule_registered = sensor_composite->register_submodule_with_params<entrystage::EntryStageSubModule>(
        *entrystage_to_foxglove_robot_bridge, 
        *entrystage_to_foxglove_alive_bridge);

    sensor_submodule_registered = sensor_composite->register_submodule_with_params<sensor::SensorSubModule>(
        info["source"], 
        info["flip"], 
        *sensor_from_serial_attitude_bridge, 
        *sensor_from_serial_robot_status_bridge);

    preprocess_submodule_registered = sensor_composite->register_submodule_with_params<detect::PreprocessSubModule>();

    detect_submodule_registered = detect_composite->register_submodule_with_params<detect::DetectSubModule>(info["model"]);

    corner_refine_submodule_registered = predict_composite->register_submodule_with_params<detect::CornerRefineSubModule>(display["detect_adjust"]);

    predict_submodule_registered = predict_composite->register_submodule_with_params<predict::MultiPolicyPredictorSubModule>(
        display["predic_debug"], display["predic_adjust"], display["tracker_adjust"]
        );

    planner_submodule_registered = predict_composite->register_submodule_with_params<plan::PlannerSubModule>(*planner_to_serial_bridge, 
        info["planner_para"], display["predic_debug"], display["predic_show"], display["predic_plot"]);

    // 设置各个任务的调试和显示选项
    timed_serial->set_debug_print(display["timed_serial_debug"]);
    std::cout<<"display timedserial_debug:"<<display["timedserial_debug"]<<std::endl;
    timed_serial->set_img_show(display["timed_serial_show"]);
    timed_serial->set_file_log(display["timed_serial_filelog"]);

    sensor_composite->set_debug_print(display["sensor_debug"]);
    sensor_composite->set_img_show(display["sensor_show"]);
    sensor_composite->set_file_log(display["sensor_filelog"]);

    detect_composite->set_debug_print(display["detect_debug"]);
    detect_composite->set_img_show(display["detect_show"]);
    detect_composite->set_file_log(display["detect_filelog"]);

    // foxglove_server->set_debug_print(display["foxglove_server_debug"]);
    // foxglove_server->set_img_show(display["foxglove_server_show"]);
    // foxglove_server->set_file_log(display["foxglove_server_filelog"]);

    predict_composite->set_debug_print(display["predic_debug"]);
    predict_composite->set_img_show(display["predic_show"]);
    predict_composite->set_file_log(display["predic_filelog"]);

    
    // 检查所有关键子模块是否注册成功
    if (!entrystage_submodule_registered 
        || !sensor_submodule_registered 
        || !preprocess_submodule_registered
        || !detect_submodule_registered 
        || !corner_refine_submodule_registered
        || !predict_submodule_registered 
        || !planner_submodule_registered
        // || !foxglove_server_independenttask_registered
        || !timed_serial_independenttask_registered) 
    {
        LOGE_S("[init] Critical modules unavailable, system cannot start");
        return false;
    }

    if (false) {
        LOGE_S("[init] some composite tasks unavailable, system still can start");
    }

    LOGM_S("[init] all composite tasks registered successfully, system can start");
    return true;
    // 第三步结束
}

int main(void)
{
    if (!init())
    {
        // 第六步第二处: 在 init 失败时释放已分配的内存
        LOGE_S("Init Fail, Quit");
        delete sensor_composite;
        delete detect_composite;
        delete predict_composite;
        delete timed_serial;
        // delete foxglove_server;
        delete planner_to_serial_bridge;
        delete entrystage_to_foxglove_robot_bridge;
        delete entrystage_to_foxglove_alive_bridge;
        delete sensor_from_serial_attitude_bridge;
        delete sensor_from_serial_robot_status_bridge;
        return 0;
        // 第六步第二处结束
    }

    const int max_mem = 4;
    // 使用新的类型别名：cap2det 采用零缓冲握手机制，其他采用有缓冲队列
    pipeline::AutoAimHandshake cap2det(0);  // 零缓冲握手
    // pipeline::AutoAimQueue cap2det(1);
    pipeline::AutoAimQueue det2pre(2);      // 有缓冲队列
    pipeline::AutoAimQueue pre2cap(max_mem + 1);  // 有缓冲队列
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(std::make_shared<ThreadDataPack>());
    }


    // 第五步: 线程管理
    // std::thread t_sensor, t_detect, t_predict, t_timed_serial, t_foxglove_server;
    std::thread t_sensor, t_detect, t_predict, t_timed_serial;

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
                            
    t_timed_serial = std::thread([&]()
                          { (*timed_serial)(); });

    // t_foxglove_server = std::thread([&]()
    //                       { (*foxglove_server)(); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    // 启动所有任务开始工作
    LOGM_S("Starting all composite tasks...");
    timed_serial->start();
    sensor_composite->start();
    detect_composite->start();
    predict_composite->start();
    // foxglove_server->start();
    LOGM_S("All composite tasks started successfully!");



    // 第五步: 停止线程
    // 阻塞轮询等待终止信号
    while (!g_stop_request.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LOGM_S("Graceful shutdown initiated...");

    // 向各个线程发出终止信号
    if (timed_serial) timed_serial->terminate();
    if (sensor_composite) sensor_composite->terminate();
    if (detect_composite) detect_composite->terminate();
    if (predict_composite) predict_composite->terminate();
    // if (foxglove_server) foxglove_server->terminate();

    // 将各个线程的 Join 任务放入监控列表
    std::vector<ThreadMonitor> monitors;

    monitors.push_back(create_monitor("TimedSerial", t_timed_serial));
    monitors.push_back(create_monitor("Sensor", t_sensor));
    monitors.push_back(create_monitor("Detect", t_detect));
    monitors.push_back(create_monitor("Predict", t_predict));
    // monitors.push_back(create_monitor("Foxglove", t_foxglove_server));

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
    // 第五步结束

    // 第六步：手动释放内存
    delete sensor_composite;
    delete detect_composite;
    delete predict_composite;
    delete timed_serial;
    // delete foxglove_server;
    delete planner_to_serial_bridge;
    delete entrystage_to_foxglove_robot_bridge;
    delete entrystage_to_foxglove_alive_bridge;
    delete sensor_from_serial_attitude_bridge;
    delete sensor_from_serial_robot_status_bridge;
    
    // 销毁 CoordTransformer 单例
    mathutils::CoordTransformer::Destroy();
    // 第六步结束

    return 0;
}
