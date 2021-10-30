//
// Inherit from auto-aim/main.cpp commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Customer - consumer model for threads io
//

#ifndef COMMON_PIPLINE_H
#define COMMON_PIPLINE_H

//submodules
#include "common.hpp"

//packages
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
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
 */
namespace pipline
{
    /**
     * @brief   线程间通信类
     * @details 用于在线程间提供缓存队列，并保证线程安全的进行读写
     * @tparam  T 用于交换的报文对象
     */
    template <typename T> // pipline for memory pool design, no thread security ensurance, make sure memory pool large enough
    class pipline_queue_t
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] _max 缓存队列的最大容量
         */
        pipline_queue_t(const int _max) : max(_max), count(0){};

        /**
         * @brief   获取报文对象
         * @details 等待缓存队列非空后将队首的报文对象出队
         * @return  指向获取的报文对象的指针
         */
        inline std::shared_ptr<T> get()
        {
            std::unique_lock<std::mutex> lock(mtx);
            while (count == 0)
                cv.wait(lock);
            count--;
            auto p = ptr_queue.front();
            ptr_queue.pop();
            cv.notify_all();
            return p;
        }

        /**
         * @brief   提交报文对象
         * @details 等待缓存队列空闲后将提交的报文对象入队
         * @param[in] p 指向提交的报文对象的指针
         */
        inline void put(std::shared_ptr<T> &p)
        {
            std::unique_lock<std::mutex> lock(mtx);
            while (count > max)
                cv.wait_for(lock, std::chrono::seconds(1));
            count++;
            ptr_queue.push(p);
            cv.notify_all();
        }
        inline void put(std::shared_ptr<T> &&p)
        {
            std::unique_lock<std::mutex> lock(mtx);
            while (count > max)
                cv.wait_for(lock, std::chrono::seconds(1));
            count++;
            ptr_queue.push(p);
            cv.notify_all();
        }

        /**
         * @brief   等待缓存队列空闲
         */
        inline void wait_for_put()
        {
            std::unique_lock<std::mutex> lock(mtx);
            while (count > max)
                cv.wait_for(lock, std::chrono::seconds(1));
        }

    private:
        int max;
        int count;
        std::mutex mtx;
        std::condition_variable cv;
        std::queue<std::shared_ptr<T> > ptr_queue;
    };
}
/**
     * @brief   线程间通信类
     */
typedef pipline::pipline_queue_t<ThreadDataPack> autoaim_pipline;
namespace pipline
{
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
         * @param[in] pipbefore 与装甲板检测的上一流程交互的 pipline
         * @param[in] pipafter  与装甲板检测的下一流程交互的 pipline
         * @note    通过 stop() 控制启停
         *          必须先进行初始化
         * @see     detect/detect.cpp\hpp detect::Detect::operator()
         */
        void operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter)
        {
        }

        /**
         * @brief   任务类初始化
         * @note    子类重载时初始化完成后应调用此函数
         * @see     detect/detect.hpp detect::Detect::init
         */
        void init()
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
        void setshow(const bool &show)
        {
            _show = show;
        }

        /**
         * @brief   设置是否显示调试信息
         */
        void setdebug(const bool &debug)
        {
            _debug = debug;
        }

    protected:
        bool _debug; /*!<标记是否显示调试信息*/
        bool _show;  /*!<标记是否展示运行结果*/
        bool _init;  /*!<标记是否完成初始化*/
        bool _run;   /*!<任务线程是否运行运行*/
    };
}

#endif //COMMON_PIPLINE_H
