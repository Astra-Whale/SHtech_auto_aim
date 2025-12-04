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
 * - 主线程阻塞在 join()，等待信号 (SIGINT/Ctrl+C)。
 *
 * 6. 【安全停机 (Safe Shutdown)】 <--- 关键安全步骤
 * - 收到信号，调用 terminate() 设置退出标志。
 * - 主线程执行 join()，确保所有子线程完全退出主循环。
 * - 此时不再有任何组件会访问 Bridge。
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
 * [ ] 4. 注册关闭 (Terminate):
 * 在 terminate() 函数中加入 `if (my_task) my_task->terminate();`
 * [ ] 5. 线程管理 (Main):
 * (a) 声明线程：`std::thread t_my_task;`
 * (b) 绑定执行：`t_my_task = std::thread([&]{ (*my_task)(); });`
 * (c) 统一启动：`my_task->start();` (在所有线程创建后)
 * (d) 等待回收：`t_my_task.join();`
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
 * [ ] 4. 注册关闭 (Terminate):
 * 在 terminate() 函数中加入 `if (my_composite) my_composite->terminate();`
 * [ ] 5. 线程管理 (Main):
 * 同 BasicTask。注意 PipelineTask 的执行函数通常需要传入流水级间管道。
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
 * [ ] 4. 注册关闭 (Terminate):
 * 无。生命周期随 PipelineTask 自动管理。
 * [ ] 5. 线程管理 (Main):
 * 无。运行在 PipelineTask 的线程中。
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
 * [ ] 4. 注册关闭 (Terminate):
 * 无。
 * [ ] 5. 线程管理 (Main):
 * 无。
 * [ ] 6. 内存释放 (Delete):
 * 在 main() 底部及 init() 异常处理块添加 `delete my_bridge;`
 * 注意：Bridge 必须在所有 Task 被 delete 后 (或至少 join 后) 才能 delete。
 *
 * ======================================================================================
 */


#include "main.hpp"

bool enemy;
bool run = false;

int totalFrameCounter = 0;

// 第一步：声明指针
// 使用通用的 PipelineTask 架构
pipeline::PipelineTask* sensor_composite = nullptr;
pipeline::PipelineTask* detect_composite = nullptr;
pipeline::PipelineTask* predict_composite = nullptr;

hardware::TimedSerial* timed_serial = nullptr;
foxgloveSer::FoxgloveServer_t* foxglove_server = nullptr;
pipeline::bridge::PlannerToSerialBridge* planner_to_serial_bridge = nullptr;

pipeline::bridge::EntryStageToFoxgloveRobotBridge* entrystage_to_foxglove_robot_bridge = nullptr;
pipeline::bridge::EntryStageToFoxgloveAliveBridge* entrystage_to_foxglove_alive_bridge = nullptr;
pipeline::bridge::SensorFromSerialAttitudeBridge* sensor_from_serial_attitude_bridge = nullptr;
pipeline::bridge::SensorFromSerialRobotStatusBridge* sensor_from_serial_robot_status_bridge = nullptr;
// 第一步结束

void terminate(int signal)
{
    LOGM_S("Received termination signal, shutting down all tasks...");

    // 第四步: 停止线程
    // 终止所有复合任务（这会唤醒等待的线程并让它们退出）
    if (timed_serial) timed_serial->terminate();
    if (sensor_composite) sensor_composite->terminate();
    if (detect_composite) detect_composite->terminate();
    if (predict_composite) predict_composite->terminate();
    if (foxglove_server) foxglove_server->terminate();
    
    LOGM_S("Quit");
    if ((timed_serial && !timed_serial->isterminated())
        ||(sensor_composite && !sensor_composite->isterminated())
        || (detect_composite && !detect_composite->isterminated())
        || (predict_composite && !predict_composite->isterminated())
        || (foxglove_server && !foxglove_server->isterminated())
    )
    {
        LOGM_S("terminate flags cannot be set, quit fail!");
        exit(0);
    }
    // 第四步结束
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
    bool detect_submodule_registered = false;
    bool predict_submodule_registered = false;
    bool planner_submodule_registered = false;
    bool timed_serial_independenttask_registered = false;
    bool foxglove_server_independenttask_registered = false;
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
    
    try{
        foxglove_server = new foxgloveSer::FoxgloveServer_t(*entrystage_to_foxglove_robot_bridge, 
                                                        *entrystage_to_foxglove_alive_bridge);
        foxglove_server_independenttask_registered = true;
    }
    catch (const std::exception &e)
    {
        LOGE_S("[init] Failed to create FoxgloveServer: %s", e.what());
        foxglove_server_independenttask_registered = false;
    }

    // 将参数和桥传递给流水线子模块，并注册它们到流水级任务
    entrystage_submodule_registered = sensor_composite->register_submodule_with_params<entrystage::EntryStageSubModule>(
        *entrystage_to_foxglove_robot_bridge, *entrystage_to_foxglove_alive_bridge);

    sensor_submodule_registered = sensor_composite->register_submodule_with_params<sensor::SensorSubModule>(
        info["source"], info["flip"], *sensor_from_serial_attitude_bridge, *sensor_from_serial_robot_status_bridge);

    detect_submodule_registered = detect_composite->register_submodule_with_params<detect::DetectSubModule>(info["model"], display["detect_adjust"]);

    predict_submodule_registered = predict_composite->register_submodule_with_params<predict::MultiPolicyPredictorSubModule>(
        info["camera_para"], atoi(info["latency"].c_str()), atoi(info["shoot_latency"].c_str()), 
        display["predic_debug"], display["predic_show"], display["predic_plot"], 
        display["predic_adjust"]);

    planner_submodule_registered = predict_composite->register_submodule_with_params<plan::PlannerSubModule>(*planner_to_serial_bridge);

    // 设置各个任务的调试和显示选项
    timed_serial->setdebug(display["timedserial_debug"]);
    timed_serial->setshow(display["timedserial_show"]);

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
        || !detect_submodule_registered 
        || !predict_submodule_registered 
        || !planner_submodule_registered
        || !foxglove_server_independenttask_registered
        || !timed_serial_independenttask_registered) 
    {
        LOGE_S("[init] Critical modules unavailable, system cannot start");
        return false;
    }

    if (false) {
        LOGE_S("[init] some composite tasks unavailable, system still can start");
    }

    LOGE_S("[init] all composite tasks registered successfully, system can start");
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
        delete foxglove_server;
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
    pipeline::AutoAimQueue det2pre(2);      // 有缓冲队列
    pipeline::AutoAimQueue pre2cap(max_mem + 1);  // 有缓冲队列
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(std::make_shared<ThreadDataPack>());
    }


    // 第五步: 线程管理
    std::thread t_sensor, t_detect, t_predict, t_timed_serial, t_foxglove_server;

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

    t_foxglove_server = std::thread([&]()
                          { (*foxglove_server)(); });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    // 启动所有任务开始工作
    LOGM_S("Starting all composite tasks...");
    timed_serial->start();
    sensor_composite->start();
    detect_composite->start();
    predict_composite->start();
    foxglove_server->start();
    LOGM_S("All composite tasks started successfully!");


    // 主线程等待所有工作线程结束
    t_timed_serial.join();
    LOGM_S("timed_serial Thread Quit Success!");

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
    // 第五步结束

    // 第六步：手动释放内存
    delete sensor_composite;
    delete detect_composite;
    delete predict_composite;
    delete timed_serial;
    delete foxglove_server;
    delete planner_to_serial_bridge;
    delete entrystage_to_foxglove_robot_bridge;
    delete entrystage_to_foxglove_alive_bridge;
    delete sensor_from_serial_attitude_bridge;
    delete sensor_from_serial_robot_status_bridge;
    // 第六步结束

    return 0;
}