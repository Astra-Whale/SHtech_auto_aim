#include "main.hpp"

// #define SHOW
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
        enemy = ENEMY_BLUE;
        LOGM_S("we are " STR_CTR(WORD_RED, "RED"));
    }
    else
    {
        enemy = ENEMY_RED;
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

    string classesFile = info["class"];
    string modelConfig = info["model"];
    string modelWeights = info["weights"];

    stdout = logfile->out;
    stderr = logfile->out;
    ArmorDetector detector(classesFile, modelConfig, modelWeights, enemy);
    stdout = screen->out;

    avg_timer t(" "), detect_a("detect"), matcp("MatCP");

    fps_counter f("Total"), send_c("Send"), receive_c("Receive"), img_get_c("imgRead"), detect_c("Detect"),
        main_c("Main"), prepro_c("pre"), track_c("track");

    struct detection_obj_t
    {
        Image image;
        std::vector<bbox_t> outs;
        std::shared_ptr<image_t> detImg;

        detection_obj_t()
        {
            image.frame = cv::Mat(video->getSize().height, video->getSize().width, CV_32FC3);
            outs.reserve(20);
        }
    };

    const int max_mem = 5;
    detection_obj_t memory_pool[max_mem];
    pipline_queue_t<detection_obj_t> cap2pre(2), pre2detect(2), detect2track(2), track2cap(max_mem + 1);
    for (int i = 0; i < max_mem; i++)
    {
        track2cap.put(&memory_pool[i]);
    }

    std::thread t_cap, t_prepare, t_detect, t_armorPos_and_send;
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

    t_cap = std::thread([&]() {
#ifdef SHOW
        cv::namedWindow("raw");
#endif // SHOW
        do
        {
            cap2pre.wait_for_put();
            detection_obj_t *obj = track2cap.get();
            bool state;
            CNT_TIM_AVG(t, { state = video->read(obj->image.frame); obj->image.index=totalFrameCounter++; }, {})
            if (!state)
            {
                LOGE(screen, "read image fail!");
                //run = false;
                LOGM(screen, "Total frames handled: %d", totalFrameCounter);
                LOGM_S("ReOpen Camera");
                video->close();
                video->init();
                track2cap.put(obj);
                //break;
            }
            if (obj->image.frame.empty())
            {
                LOGW(screen, "empty image");
                track2cap.put(obj);
                continue;
            }
            CNT_FPS(img_get_c, {});
#ifdef SHOW
            cv::Mat show;
            obj->image.frame.copyTo(show);
            cv::resize(show, show, cv::Size(256, 256));
            cv::imshow("raw", show);
            cv::waitKey(1);
#endif
            cap2pre.put(obj);
        } while (run);
    });

    t_prepare = std::thread([&]() {
        do
        {
            pre2detect.wait_for_put();
            detection_obj_t *obj = cap2pre.get();
            obj->detImg = detector.detector->mat_to_image_resize(obj->image.frame);
            pre2detect.put(obj);
            CNT_FPS(prepro_c, {});
        } while (run);
    });

    t_detect = std::thread([&]() {
        do
        {
            detect2track.wait_for_put();
            detection_obj_t *obj = pre2detect.get();
            CNT_TIM_AVG(detect_a, { detector.detect(obj->detImg, obj->image.frame, obj->outs); }, {});
            CNT_FPS(detect_c, {});
            detect2track.put(obj);
        } while (run);
    });

    t_armorPos_and_send = std::thread([&]() {
        DetectResult out, out_in_world;
        std::shared_ptr<tracker> track = std::make_shared<tracker>(enemy, std::atof(info["robot_id"].c_str()), 150, 100);

        do
        {
            track2cap.wait_for_put();
            detection_obj_t *obj = detect2track.get();
            c.receive();
            //gim_state.shoot_speed = 10; //for debug
            int det_res = track->predict(obj->image.frame, obj->outs, out);
            CNT_FPS(receive_c, { LOGM(screen, "[IMU] Yaw Pitch Shoot: (%.1f,%.1f,%.1f)", gim_state.curr_yaw / 3.1415 * 180, gim_state.curr_pitch / 3.1415 * 180, gim_state.shoot_speed); });
            if (det_res == DESTROY)
            {
                LOGM_F("New Tracker");
                track = std::make_shared<tracker>(enemy, std::atof(info["robot_id"].c_str()), 150, 100);
            }
            else if (det_res == PREDICT)
            {
                c.transmit(out.ypr.x, out.ypr.y, out.dist);
                //LOGM(screen, "[SEND] Yaw Pitch T Dist:(%.1f,%.1f,%.1f,%.1f)",
                //     out.ypr.x, out.ypr.y, out.ypr.z, out.dist);
                LOGM_F("[SEND] Yaw:(%.1f,%d)", out.ypr.x, obj->image.index);
                LOGM_F("[SEND] Pitch:(%.1f,%d)", out.ypr.y, obj->image.index);
                CNT_FPS(send_c, { LOGM(screen, "[SEND] Yaw Pitch T Dist:(%.1f,%.1f,%.1f,%.1f)",
                                       out.ypr.x, out.ypr.y, out.ypr.z, out.dist); });
            }
            CNT_FPS(track_c, {});
            track2cap.put(obj);
        } while (run);
    });

    pthread_sigmask(SIG_SETMASK, &oldmask, NULL);

    t_cap.join();
    LOGM_S("Read Thread Quit Success!");
    t_prepare.join();
    LOGM_S("Prepare Thread Quit Success!");
    t_detect.join();
    LOGM_S("Detect Thread Quit Success!");
    t_armorPos_and_send.join();
    LOGM_S("Predict Thread Quit Success!");

    std::cout << "finish join" << std::endl;

    clear();

    LOGM(screen, "Successfully Quit!");

    return 0;
}
