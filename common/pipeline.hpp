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
 *    ├── 职责：流水级管理，线程生命周期控制
 *    ├── 地位：独立的处理单元，可单独存在
 *    └── 关系：与其他 BasicTask 形成并列的流水线段
 * 
 * CompositeTask (组合层) 
 *    ├── 继承：BasicTask (IS-A 关系)
 *    ├── 职责：多submodule的协调管理
 *    └── 关系：在流水线中等价于 BasicTask
 * 
 * SubModule (功能层)
 *    ├── 职责：具体功能实现，无线程管理
 *    └── 关系：被 CompositeTask 组合、拥有和管理
 * 
 * 【流水线中存在形式】
 * BasicTask_A → BasicTask_B → CompositeTask_C → BasicTask_D
 *                              ├── owns → SubModule_1
 *                              ├── owns → SubModule_2  
 *                              └── owns → SubModule_3
 * 
 * ================================================================================================
 */
namespace pipeline
{
    class BasicTask;

    /**
     * @brief   线程间通信类
     * @details 用于在线程间提供缓存队列，并保证线程安全的进行读写
     * @tparam  T 用于交换的报文对象
     */
    template <typename T> // pipeline for memory pool design, no thread security ensurance, make sure memory pool large enough
    class pipeline_queue_t
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] _max 缓存队列的最大容量
         */
        pipeline_queue_t(const int _max) : max(_max), count(0){};

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
     * @brief   线程间通信类
     */
    using autoaim_pipeline = pipeline_queue_t<ThreadDataPack>;
    
    /**
     * @brief   任务类的基类
     */
    class BasicTask
    {
    public:
        BasicTask() : _debug(false), _show(false), _init(false), _run(true) {}
        ~BasicTask()
        {
            stop();
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
         * @note    通过 stop() 控制启停
         *          必须先进行初始化
         * @see     detect/detect.cpp\hpp detect::Detect::operator()
         */
        virtual void operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter)
        {
        }

        /**
         * @brief   任务类初始化
         * @note    子类重载时初始化完成后应调用此函数
         * @see     detect/detect.hpp detect::Detect::init
         */
        virtual void init()
        {
            _init = true;
        }

        /**
         * @brief   停止任务线程
         */
        void stop(void)
        {
            _run = false;
        }

        /**
         * @brief   设置是否展示运行结果
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

        bool isalive() const
        {
            return _init && _run;
        }

    protected:
        bool _debug; /*!<标记是否显示调试信息*/
        bool _show;  /*!<标记是否展示运行结果*/
        bool _init;  /*!<标记是否完成初始化*/
        bool _run;   /*!<任务线程是否运行运行*/
    };
    
    /**
     * @brief   子模块基类
     * @details 提供与 BasicTask 类似的接口，但更轻量化
     */
    class SubModule
    {
    public:
        SubModule() : _init(false), _debug(false), _show(false) {}
        virtual ~SubModule() = default;

        // 禁用复制，只允许移动
        SubModule(const SubModule&) = delete;
        SubModule& operator=(const SubModule&) = delete;
        SubModule(SubModule&&) = default;
        SubModule& operator=(SubModule&&) = default;

        /**
         * @brief   子模块初始化
         */
        virtual void init() { _init = true; }

        /**
         * @brief   设置是否显示调试信息
         */
        virtual void setdebug(bool debug) { _debug = debug; }

        /**
         * @brief   设置是否展示运行结果
         */
        virtual void setshow(bool show) { _show = show; }

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        virtual bool process(std::shared_ptr<ThreadDataPack>& data, 
                           BasicTask* parent) = 0;

        bool is_initialized() const { return _init; }

    protected:
        bool _init;   /*!<标记是否完成初始化*/
        bool _debug;  /*!<标记是否显示调试信息*/
        bool _show;   /*!<标记是否展示运行结果*/
    };

     /**
     * @brief   复合任务类
     * @details 管理多个子模块的执行
     */
    class CompositeTask : public BasicTask
    {
    public:
        /**
         * @brief   注册子模块
         * @param[in] submodule 子模块的独占所有权，调用后 submodule 将被移动
         */
        void register_submodule(std::unique_ptr<SubModule> submodule)
        {
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
         * @brief   初始化所有子模块
         */
        void init() override
        {
            for (auto& submodule : submodules)
            {
                submodule->init();
            }
            BasicTask::init();
        }

        /**
         * @brief   设置调试信息显示（级联到所有子模块）
         */
        void setdebug(bool debug) override
        {
            BasicTask::setdebug(debug);
            for (auto& submodule : submodules)
            {
                submodule->setdebug(debug);
            }
        }

        /**
         * @brief   设置结果展示（级联到所有子模块）
         */
        void setshow(bool show) override
        {
            BasicTask::setshow(show);
            for (auto& submodule : submodules)
            {
                submodule->setshow(show);
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
         * @brief   检查所有子模块是否已初始化
         */
        bool all_submodules_initialized() const
        {
            for (const auto& submodule : submodules)
            {
                if (!submodule->is_initialized())
                {
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief   获取子模块的初始化状态
         * @param[in] index 子模块索引
         * @return 初始化状态，索引无效时返回 false
         */
        bool get_submodule_init_status(size_t index) const
        {
            if (index >= submodules.size())
                return false;
            return submodules[index]->is_initialized();
        }

        /**
         * @brief   执行所有子模块
         */
        void operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter) override
        {
            if (submodules.empty())
            {
                // 如果没有子模块，直接传递数据
                while (isalive())
                {
                    auto data = pipebefore.get(this);
                    if (data)
                    {
                        pipeafter.put(data, this);
                    }
                }
                return;
            }

            // 主处理循环
            while (isalive())
            {
                // 从输入管道获取数据
                auto data = pipebefore.get(this);
                if (!data)
                    break;

                // 串行执行所有子模块，直接处理数据
                bool should_continue = true;
                for (size_t i = 0; i < submodules.size() && should_continue; ++i)
                {
                    should_continue = submodules[i]->process(data, this);
                }
                
                // 只有当所有子模块都成功处理时才传递到下一阶段
                if (should_continue)
                {
                    pipeafter.put(data, this);
                }
                else
                {
                    // 如果处理失败，将数据包重新放回输入队列
                    pipebefore.put(data, this);
                }
            }
        }

    private:
        std::vector<std::unique_ptr<SubModule>> submodules;
    };

    template<typename T>
    inline bool pipeline_queue_t<T>::wait_for_put(BasicTask* employee)
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
    inline bool pipeline_queue_t<T>::wait_for_get(BasicTask* employee)
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
