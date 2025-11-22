//
// Inherit from auto-aim/main.cpp commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Customer - consumer model for threads io
//

#ifndef COMMON_pipeline_H
#define COMMON_pipeline_H

//submodules
#include "common.hpp"

//packages
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <stdexcept>
#include <type_traits>
/**
 * @brief   用于安全释放指针的函数类
 */
struct SafeDeleter
{
    template <typename T>
    void operator()(T *obj) const
    {
        delete obj;
    }
};

/**
 * @brief   释放安全的 unique_ptr 智能指针
 */
template <typename T>
using SafeUniquePtr = std::unique_ptr<T, SafeDeleter>;

/**
 * @brief   用于线程间通信和多任务管理的命名空间
 * @details 实现了线程间通信类
 *          定义了任务基类并实现了基本管理函数
 *          定义了线程间通信报文
 * ================================================================================================
 * 
 * 【架构层次】
 * 
 * BasicTask (基础层)
 *    ├── 职责：线程生命周期控制、调试变量控制
 *    ├── 地位：独立的处理单元，只可单独存在，没有流水线支持
 *    └── 关系：独立运行的流水线外单元：如cboard
 * 
 * PipelineTask (组合层) 
 *    ├── 继承：BasicTask (IS-A 关系)
 *    ├── 职责：提供流水线管线（Pipeline类）相关支持，submodule的协调管理
 *    └── 关系：线程关系上的流水单元，拥有多个 SubModule
 * 
 * SubModule (功能层)
 *    ├── 职责：具体功能实现，无线程管理。只有调试变量和process函数
 *    └── 关系：被 PipelineTask 组合、拥有和管理
 * 
 * 【流水线中存在形式】
 * PipelineTask_A       →       PipelineTask_B       →       PipelineTask_C
 *  ├── owns → SubModule_1       ├── owns → SubModule_4       ├── owns → SubModule_5
 *  ├── owns → SubModule_2
 *  └── owns → SubModule_3
 *  （独立的） BasicTask_D
 * ================================================================================================
 */
namespace pipeline
{
    class BasicTask;

    /**
     * @brief   线程间通信类（有缓冲队列）
     * @details 用于在线程间提供缓存队列，并保证线程安全的进行读写
     * @tparam  T 用于交换的报文对象
     */
    template <typename T> // pipeline for memory pool design, no thread security ensurance, make sure memory pool large enough
    class BufferedPipeline
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] _max 缓存队列的最大容量
         */
        BufferedPipeline(const int _max) : max(_max), count(0){};

        /**
         * @brief   获取报文对象
         * @details 等待缓存队列非空后将队首的报文对象出队
         * @return  指向获取的报文对象的指针
         */
        inline std::shared_ptr<T> get(BasicTask* employee = nullptr)
        {
            bool stat = wait_for_get(employee);
            if (stat && count > 0)
            {
                count--;
                auto p = ptr_queue.front();
                ptr_queue.pop();
                cv.notify_all();
                return p;
            }
            std::cout<<"employee dead"<<std::endl;
            return nullptr;
        }

        /**
         * @brief   提交报文对象
         * @details 等待缓存队列空闲后将提交的报文对象入队
         * @param[in] p 指向提交的报文对象的指针
         */
        inline bool put(std::shared_ptr<T> &&p, BasicTask* employee = nullptr)
        {
            bool stat = wait_for_put(employee);
            if (!stat || count < max)
            {
                count++;
                ptr_queue.push(std::move(p));
                cv.notify_all();
                return true;
            }
            return false;
        }

        /**
         * @brief   提交报文对象
         * @details 等待缓存队列空闲后将提交的报文对象入队
         * @param[in] p 指向提交的报文对象的指针
         */
        inline bool put(std::shared_ptr<T> &p, BasicTask* employee = nullptr)
        {
            bool stat = wait_for_put(employee);
            if (!stat || count < max)
            {
                count++;
                ptr_queue.push(p);
                cv.notify_all();
                return true;
            }
            return false;
        }

    private:
        int max;
        int count;
        std::mutex mtx;
        std::condition_variable cv;
        std::queue<std::shared_ptr<T> > ptr_queue;

        /**
         * @brief   等待缓存队列空闲
         */
        inline bool wait_for_put(BasicTask* employee);

        /**
         * @brief   等待缓存队列空闲
         */
        inline bool wait_for_get(BasicTask* employee);
    };

    /**
     * @brief   零缓冲握手管道类
     * @details 实现严格的生产者-消费者同步握手机制，无数据积压
     * @tparam  T 用于交换的报文对象
     */
    template <typename T>
    class HandshakePipeline
    {
    public:
        /**
         * @brief   构造函数（为兼容性接受容量参数但忽略）
         * @param[in] ignored_max 容量参数（握手模式下忽略）
         */
        HandshakePipeline(const int ignored_max) {}

        /**
         * @brief   获取报文对象（消费者）
         * @details 等待生产者提交数据后取走数据并完成握手
         * @return  指向获取的报文对象的指针
         */
        inline std::shared_ptr<T> get(BasicTask* employee = nullptr)
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            // 标记消费者准备好
            ready_to_get = true;
            cv_put.notify_one();  // 通知生产者可以提交
            
            // 等待生产者提交数据
            if (employee != nullptr)
            {
                while (!data_ready && employee->isalive())
                {
                    cv_get.wait_for(lock, std::chrono::milliseconds(100));
                }
                if (!employee->isalive())
                {
                    return nullptr;
                }
            }
            else
            {
                cv_get.wait(lock, [this]() { return data_ready; });
            }
            
            // 取走数据
            auto result = std::move(data_to_transfer);
            data_to_transfer = nullptr;
            data_ready = false;
            ready_to_get = false;
            
            return result;
        }

        /**
         * @brief   提交报文对象（生产者，右值引用版本）
         * @details 等待消费者准备好后提交数据并完成握手
         * @param[in] p 指向提交的报文对象的指针
         */
        inline bool put(std::shared_ptr<T> &&p, BasicTask* employee = nullptr)
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            // 等待消费者准备好
            if (employee != nullptr)
            {
                while (!ready_to_get && employee->isalive())
                {
                    cv_put.wait_for(lock, std::chrono::milliseconds(100));
                }
                if (!employee->isalive())
                {
                    return false;
                }
            }
            else
            {
                cv_put.wait(lock, [this]() { return ready_to_get; });
            }
            
            // 提交数据
            data_to_transfer = std::move(p);
            data_ready = true;
            cv_get.notify_one();  // 通知消费者数据已就绪
            
            return true;
        }

        /**
         * @brief   提交报文对象（生产者，左值引用版本）
         * @details 等待消费者准备好后提交数据并完成握手
         * @param[in] p 指向提交的报文对象的指针
         */
        inline bool put(std::shared_ptr<T> &p, BasicTask* employee = nullptr)
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            // 等待消费者准备好
            if (employee != nullptr)
            {
                while (!ready_to_get && employee->isalive())
                {
                    cv_put.wait_for(lock, std::chrono::milliseconds(100));
                }
                if (!employee->isalive())
                {
                    return false;
                }
            }
            else
            {
                cv_put.wait(lock, [this]() { return ready_to_get; });
            }
            
            // 提交数据
            data_to_transfer = p;
            data_ready = true;
            cv_get.notify_one();  // 通知消费者数据已就绪
            
            return true;
        }

    private:
        std::mutex mtx;
        std::condition_variable cv_put;  /*!< 生产者等待/消费者通知 */
        std::condition_variable cv_get;  /*!< 消费者等待/生产者通知 */
        std::shared_ptr<T> data_to_transfer;  /*!< 临时数据交换点 */
        bool ready_to_get = false;  /*!< 消费者准备好标记 */
        bool data_ready = false;    /*!< 生产者提交数据标记 */
    };

    /**
     * @brief   类型别名定义
     */
    template <typename T> using AutoAimQueueT = BufferedPipeline<T>;
    template <typename T> using AutoAimHandshakeT = HandshakePipeline<T>;
    
    using AutoAimQueue = AutoAimQueueT<ThreadDataPack>;
    using AutoAimHandshake = AutoAimHandshakeT<ThreadDataPack>;
    
    /**
     * @brief   向后兼容别名（指向有缓冲队列）
     */
    using autoaim_pipeline = AutoAimQueue;
    
    /**
     * @brief   任务类的基类，现在原则上不允许直接插入流水线
     */
    class BasicTask
    {
    public:
        BasicTask() : _debug(false), _show(false), _should_run(false), _should_terminate(false) {}
        virtual ~BasicTask()
        {
            terminate();
        }

        /**
         * @brief   禁用拷贝构造
         */
        BasicTask(const BasicTask &) = delete;

        /**
         * @brief   禁用拷贝构造
         */
        BasicTask operator=(const BasicTask &) = delete;

        /**
         * @brief   任务线程入口
         * @details 未实例化
         * @param[in] pipebefore 与装甲板检测的上一流程交互的 pipeline
         * @param[in] pipeafter  与装甲板检测的下一流程交互的 pipeline
         * @note    通过 start() stop() terminate() 控制运行，暂停，终止
         */
        virtual void operator()()
        {
        }

        /**
         * @brief   启动任务（允许任务运行）并通知等待的线程
         */
        virtual void start(void)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                _should_run = true;
            }
            state_cv.notify_all();  // 唤醒等待的线程进入工作循环
        }

        /**
         * @brief   停止任务（暂停运行，但不终止）
         * @note    线程会自然退出到等待状态，无需通知
         */
        virtual void stop(void)
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            _should_run = false;
        }

        /**
         * @brief   终止任务（彻底结束任务）并通知等待的线程
         */
        virtual void terminate(void)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                _should_run = false;
                _should_terminate = true;
            }
            state_cv.notify_all();  // 唤醒等待的线程让其检查终止条件并退出
        }

        /**
         * @brief   重置终止状态（允许重新启动）
         */
        void reset(void)
        {
            _should_terminate = false;
            _should_run = true;
        }

        /**
         * @brief   设置是否展示调试画面
         */
        virtual void setshow(const bool &show)
        {
            _show = show;
        }

        /**
         * @brief   设置是否显示调试信息
         */
        virtual void setdebug(const bool &debug)
        {
            _debug = debug;
        }

        /**
         * @brief   任务是否应该保持活跃（既要运行且未被终止）
         * @return  bool 当应该运行且未被终止时任务保持活跃
         */
        bool isalive() const
        {
            return _should_run && !_should_terminate;
        }

        /**
         * @brief   任务是否被终止
         * @return  bool 任务是否已被终止
         */
        bool isterminated() const
        {
            return _should_terminate;
        }

    protected:
        /**
         * @brief   等待任务状态变更的统一方法
         * @details 等待 start() 或 terminate() 信号，通过返回值指示下一步行为
         * @return  true - 应该开始/继续工作；false - 应该终止线程
         */
        bool wait_for_state_change()
        {
            std::unique_lock<std::mutex> lock(state_mutex);
            state_cv.wait(lock, [this]() {
                return _should_run || _should_terminate;
            });
            return _should_run && !_should_terminate;  // 只有在应该运行且未终止时返回true
        }
        bool _debug;            /*!< 标记是否显示调试信息 */
        bool _show;             /*!< 标记是否展示运行结果 */
        bool _should_run;       /*!< 外部控制：任务是否应该运行 */
        bool _should_terminate; /*!< 外部控制：任务是否应该被彻底终止 */
        
        std::mutex state_mutex;                    /*!< 状态变更的互斥锁 */
        std::condition_variable state_cv;          /*!< 状态变更的条件变量 */
    };
    
    /**
     * @brief   子模块基类
     * @details 提供与 BasicTask 类似的接口，但更轻量化，不管理线程
     *          只能被 PipelineTask 类拥有和管理
     */
    class SubModule
    {
    public:
        SubModule(SubModuleName name) : _debug(false), _show(false), submodule_name(name) {}
        virtual ~SubModule() = default;

        // 禁用复制，只允许移动
        SubModule(const SubModule&) = delete;
        SubModule& operator=(const SubModule&) = delete;
        SubModule(SubModule&&) = default;
        SubModule& operator=(SubModule&&) = default;



        /**
         * @brief   设置是否显示调试信息
         */
        virtual void setdebug(const bool &debug) { _debug = debug; }

        /**
         * @brief   设置是否展示调试画面
         */
        virtual void setshow(const bool &show) { _show = show; }

        /**
         * @brief   获取子模块名称（枚举类型)
         */
        SubModuleName get_submodule_name() const { return submodule_name; }

        virtual bool should_skip(std::shared_ptr<ThreadDataPack> data) const { return false; }

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  SubModuleResult         返回 SUCCESS 表示成功，FAILURE 表示失败，此处不应返回SKIP
         */
        virtual SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                           const BasicTask* parent) = 0;

    protected:
        bool _debug;  /*!<标记是否显示调试信息*/
        bool _show;   /*!<标记是否展示运行结果*/
        SubModuleName submodule_name; /*!< 子模块名称 */
    };

     /**
     * @brief   复合任务类
     * @details 管理多个子模块的执行
     */
    class PipelineTask : public BasicTask
    {
    public:
        /**
         * @brief   子模块注册器模板类
         * @tparam T 子模块类型，必须继承自 SubModule
         */
        template<typename T>
        class SubModuleRegistrar
        {
        public:
            /**
             * @brief   构造函数，注册并初始化子模块
             * @tparam Args 构造函数参数类型
             * @param task 复合任务引用
             * @param args 构造函数参数
             */
            template<typename... Args>
            SubModuleRegistrar(PipelineTask& task, Args&&... args)
            {
                task.register_submodule_with_params<T>(std::forward<Args>(args)...);
            }
        };
        /**
         * @brief   注册子模块
         * @param[in] submodule 子模块的独占所有权，调用后 submodule 将被移动
         * @throws  std::invalid_argument 当 submodule 为 nullptr 时抛出异常
         */
        void register_submodule(std::unique_ptr<SubModule> submodule)
        {
            if (!submodule)
            {
                throw std::invalid_argument("PipelineTask: Cannot register null submodule");
            }
            submodules.emplace_back(std::move(submodule));
        }

        /**
         * @brief   使用初始化列表批量注册子模块
         * @param[in] submodule_list 子模块初始化列表
         */
        template<typename... Args>
        void register_submodules(Args&&... args)
        {
            static_assert(sizeof...(args) > 0, "At least one submodule must be provided");
            (register_submodule(std::forward<Args>(args)), ...);
        }

        /**
         * @brief   注册并初始化子模块（模板版本）
         * @tparam T 子模块类型，必须继承自 SubModule
         * @tparam Args 构造函数参数类型
         * @param args 构造函数参数
         * @return bool 注册是否成功
         */
        template<typename T, typename... Args>
        bool register_submodule_with_params(Args&&... args)
        {
            // 检查 T 是否继承自 SubModule
            static_assert(std::is_base_of<SubModule, T>::value, "T must be derived from SubModule");

            // 检查是否可以用提供的参数构造 T
            static_assert(std::is_constructible<T, Args...>::value, "Cannot construct T with provided arguments");

            // 尝试构造子模块
            try {
                auto submodule = std::make_unique<T>(std::forward<Args>(args)...);
                register_submodule(std::move(submodule));
                return true;
            } catch (const std::exception& e) {
                // 构造失败，打印错误信息并返回false
                //LOGE_S("Failed to construct submodule: %s", e.what());
                return false;
            } catch (...) {
                // 其他异常，打印错误信息并返回false
                //LOGE_S("Failed to construct submodule with provided arguments");
                return false;
            }
        }

        PipelineTask() : BasicTask() {};
        virtual ~PipelineTask() {

            if (!submodules.empty() && submodules[0] != nullptr) {
        
                std::cout<<static_cast<uint8_t>(submodules[0]->get_submodule_name()) << std::endl;
        
            }
            else {
        
                std::cout<<"submodules empty"<<std::endl;
        
            }
        
        }




        /**
         * @brief   设置调试信息显示（级联到所有子模块）
         */
        virtual void setdebug(const bool &debug) override
        {
            BasicTask::setdebug(debug);
            for (auto& submodule : submodules)
            {
                if(submodule)
                {
                    submodule->setdebug(debug);
                }
            }
        }

        /**
         * @brief   设置结果展示（级联到所有子模块）
         */
        virtual void setshow(const bool &show) override
        {
            BasicTask::setshow(show);
            for (auto& submodule : submodules)
            {
                if(submodule)
                {
                    submodule->setshow(show);
                }
            }
        }

        /**
         * @brief   获取子模块数量
         */
        size_t get_submodule_count() const
        {
            return submodules.size();
        }



        /**
         * @brief   执行所有子模块（模板版本，支持不同管道类型）
         * @tparam InputPipe 输入管道类型
         * @tparam OutputPipe 输出管道类型
         */
        template<typename InputPipe, typename OutputPipe>
        void operator()(InputPipe &pipebefore, OutputPipe &pipeafter)
        {
            // 统一的等待-工作循环
            while (true)
            {
                // 等待启动信号或终止信号
                if (!wait_for_state_change())
                {
                    break;  // 收到终止信号，退出线程
                }
                
                // 收到启动信号，开始工作循环
                while (isalive())
                {
                    // 从输入管道获取数据
                    auto data = pipebefore.get(this);
                    if (!data)
                        break;

                    // 如果没有子模块，直接传递数据（透传模式）
                    if (submodules.empty())
                    {
                        pipeafter.put(data, this);
                        continue;
                    }

                    // 串行执行所有子模块，直接处理数据
                    // 由于注册时已保证所有子模块都有效，不需要再检查 nullptr
                    // 先调用 should_skip 决定是否跳过子模块
                    // 记录了子模块的开始和结束时间戳，如果跳过则结束和开始时间相同
                    // 通过 process 执行子模块处理，返回值记录处理结果
                    for (auto& submodule : submodules)
                    {
                        if(submodule->should_skip(data))
                        {
                            data->submodule_results[static_cast<uint8_t>(submodule->get_submodule_name())] = SubModuleResult::SKIP;
                            data->submodule_timestamps[static_cast<uint8_t>(submodule->get_submodule_name())] = 
                                std::make_pair(std::chrono::steady_clock::now(), std::chrono::steady_clock::now());
                            continue;
                        }
                        auto t_start =  std::chrono::steady_clock::now();
                        auto result = submodule->process(data, this);
                        auto t_end = std::chrono::steady_clock::now();
                        data->submodule_results[static_cast<uint8_t>(submodule->get_submodule_name())] = result;
                        data->submodule_timestamps[static_cast<uint8_t>(submodule->get_submodule_name())] = 
                            std::make_pair(t_start, t_end);
                    }
                    
                    pipeafter.put(data, this);
                }
                
                // 工作循环结束（被stop），回到等待状态
            }
        }

    private:
        std::vector<std::unique_ptr<SubModule>> submodules;
    };

    template<typename T>
    inline bool BufferedPipeline<T>::wait_for_put(BasicTask* employee)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (employee != nullptr)
        {
            while (count >= max && employee->isalive())
            {
                cv.wait_for(lock, std::chrono::seconds(1));
            }
            return employee->isalive();
        }
        else
        {
            while (count >= max)
                cv.wait_for(lock, std::chrono::seconds(1));
            return true;
        }
    }

    template<typename T>
    inline bool BufferedPipeline<T>::wait_for_get(BasicTask* employee)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (employee != nullptr)
        {
            while (count == 0 && employee->isalive())
            {
                cv.wait_for(lock, std::chrono::seconds(1)); 
            }
            return employee->isalive();
        }
        else
        {
            while (count == 0)
                cv.wait_for(lock, std::chrono::seconds(1));
            return true;
        }
    }
}

#endif //COMMON_pipeline_H
