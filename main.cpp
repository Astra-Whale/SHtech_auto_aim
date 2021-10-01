#include "main.hpp"
#include "video_wrapper.h"
#include "hikcam_wrapper.h"
//#define SHOW
template <typename T> // pipline for memory pool design, no thread security ensurance, make sure memory pool large enough
class pipline_queue_t
{
public:
    pipline_queue_t(const int _max) : max(_max), count(0){};
    inline T *get()
    {
        std::unique_lock<std::mutex> lock(mtx);
        while (count == 0)
            cv.wait(lock);
        count--;
        T *p = ptr_queue.front();
        ptr_queue.pop();
        cv.notify_all();
        return p;
    }
    inline void put(T *p)
    {
        std::unique_lock<std::mutex> lock(mtx);
        while (count > max)
            cv.wait_for(lock, std::chrono::seconds(1));
        count++;
        ptr_queue.push(p);
        cv.notify_all();
    }
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
    std::queue<T *> ptr_queue;
};

WrapperHead *video = nullptr;
cmd_parser parser;
map<string, string> info;
map<string, bool> display;
Comm c;
bool enemy;
bool run = false;

int totalFrameCounter = 0;

void clear(void)
{
    if (video)
    {
        delete video;
    }
}

void stop(int signal)
{
    run = false;
    LOGM_S("Quit");
}

bool init(void)
{
    //signal(SIGINT, stop);
    //signal(SIGSEGV, stop);
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
    if (info["enemy"] == "BLUE")
    {
        enemy = 0;
        LOGM_S("we are " STR_CTR(WORD_RED, "RED"));
    }
    else
    {
        enemy = 1;
        LOGM_S("we are " STR_CTR(WORD_BLUE, "BLUE"));
    }
    if (info["source"] == "0")
    {
        video = new HikCamWrapper();
    }
    else
    {
        video = new VideoWrapper(info["source"]);
    }
    if (!video->init())
    {
        return false;
    }
    if (info["port"] != "None")
    {
        c.open(info["port"]);
    }
    return true;
}

int main(void)
{

    if (!init())
    {
        clear();
        LOGE(screen, "Init Fail, Quit");
        return 0;
    }
    run = true;

    string modelConfig = info["model"];

    TRTModule detector(modelConfig);

    struct detection_obj_t
    {
        cv::Mat frame;
        std::vector<bbox_t> bboxes;
        int index;

        detection_obj_t()
        {
        }
    };

    const int max_mem = 5;
    detection_obj_t memory_pool[max_mem];
    pipline_queue_t<detection_obj_t> cap2det(2), det2pre(2), pre2cap(max_mem + 1);
    for (int i = 0; i < max_mem; i++)
    {
        pre2cap.put(&memory_pool[i]);
    }

    std::thread t_cap, t_detect, t_predict;
    //camera cap thread

    //cv::Mat dummy;
    //video->read(dummy);
    //cv::imshow("a", dummy); // accelerator

    sigset_t oldmask;
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

    t_cap = std::thread([&]()
                        {
                            do
                            {
                                cap2det.wait_for_put();
                                detection_obj_t *obj = pre2cap.get();
                                bool state;
                                                state = video->read(obj->frame);
                                                obj->index = totalFrameCounter++;
                                if (!state)
                                {
                                    LOGE(screen, "read image fail!");
                                    //run = false;
                                    LOGM(screen, "Total frames handled: %d", totalFrameCounter);
                                    LOGM_S("ReOpen Camera");
                                    video->close();
                                    video->init();
                                    pre2cap.put(obj);
                                    //break;
                                }
                                if (obj->frame.empty())
                                {
                                    LOGW(screen, "empty image");
                                    pre2cap.put(obj);
                                    continue;
                                }
                                cap2det.put(obj);
                            } while (run);
                        });

    t_detect = std::thread([&]()
                           {
                               const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
                               do
                               {
                                   det2pre.wait_for_put();
                                   detection_obj_t *obj = cap2det.get();
                                   obj->bboxes = detector(obj->frame);
                                   cv::Mat im2show = obj->frame.clone();
                                   for (const auto &b : obj->bboxes)
                                   {
                                       cv::line(im2show, b.pts[0], b.pts[1], colors[2], 2);
                                       cv::line(im2show, b.pts[1], b.pts[2], colors[2], 2);
                                       cv::line(im2show, b.pts[2], b.pts[3], colors[2], 2);
                                       cv::line(im2show, b.pts[3], b.pts[0], colors[2], 2);
                                       cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
                                   }
                                   cv::imshow("", im2show);
                                   cv::waitKey(1);
                                   det2pre.put(obj);
                               } while (run);
                           });

    t_predict = std::thread([&]()
                            {
                                float cx = 6.178e2;
                                float cy = 0;
                                float fx = 2.191e3;
                                float fy = 4.063e2;const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};

                                do
                                {
                                    pre2cap.wait_for_put();
                                    detection_obj_t *obj = det2pre.get();

                                    auto detections = obj->bboxes;
                                    if(detections.size()==0)
                                    {
                                        pre2cap.put(obj);
                                        continue;
                                    }
                                    std::sort(detections.begin(), detections.end(), [](const auto &bx, const auto &by)
                                              {
                                                  auto bx_a = sqrt(pow(bx.pts[0].x - bx.pts[1].x, 2) + pow(bx.pts[0].y - bx.pts[1].y, 2));
                                                  auto bx_b = sqrt(pow(bx.pts[1].x - bx.pts[2].x, 2) + pow(bx.pts[1].y - bx.pts[2].y, 2));
                                                  auto bx_c = sqrt(pow(bx.pts[2].x - bx.pts[3].x, 2) + pow(bx.pts[2].y - bx.pts[3].y, 2));
                                                  auto bx_d = sqrt(pow(bx.pts[3].x - bx.pts[0].x, 2) + pow(bx.pts[3].y - bx.pts[0].y, 2));
                                                  auto bx_z = (bx_a + bx_b + bx_c + bx_d) / 2;
                                                  auto bx_size = 2 * sqrt((bx_z - bx_a) * (bx_z - bx_b) * (bx_z - bx_c) * (bx_z - bx_d));
                                                  auto by_a = sqrt(pow(bx.pts[0].x - bx.pts[1].x, 2) + pow(bx.pts[0].y - bx.pts[1].y, 2));
                                                  auto by_b = sqrt(pow(bx.pts[1].x - bx.pts[2].x, 2) + pow(bx.pts[1].y - bx.pts[2].y, 2));
                                                  auto by_c = sqrt(pow(bx.pts[2].x - bx.pts[3].x, 2) + pow(bx.pts[2].y - bx.pts[3].y, 2));
                                                  auto by_d = sqrt(pow(bx.pts[3].x - bx.pts[0].x, 2) + pow(bx.pts[3].y - bx.pts[0].y, 2));
                                                  auto by_z = (by_a + by_b + by_c + by_d) / 2;
                                                  auto by_size = 2 * sqrt((by_z - by_a) * (by_z - by_b) * (by_z - by_c) * (by_z - by_d));
                                                  return bx_size > by_size;
                                              });
                                    auto &target = detections.front();
                                    float x = (target.pts[0].x + target.pts[1].x + target.pts[2].x + target.pts[3].x) / 4.f;
                                    float y = (target.pts[0].y + target.pts[1].y + target.pts[2].y + target.pts[3].y) / 4.f;
                                    detection_t robot_cmd;
                                    robot_cmd.pit = (float)atan2(y - cy, fy)-1.57f/2;
                                    robot_cmd.yaw = (float)atan2(x - cx, fx);
                                    robot_cmd.dist = 0;
                                    robot_cmd.shoot = 0;
printf("(%.2f,%.2f)\n",robot_cmd.pit,robot_cmd.yaw);
                                    pre2cap.put(obj);
                                } while (run);
                            });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    t_cap.join();
    LOGM_S("Read Thread Quit Success!");
    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    t_predict.join();
    LOGM_S("Predict Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    clear();

    LOGM(screen, "Successfully Quit!");

    return 0;
}
