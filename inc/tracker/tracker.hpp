#ifndef _TRACKER_HPP_
#define _TRACKER_HPP_
//Opencv
#include <opencv2/opencv.hpp>
//Std
#include <vector>
//ArmorPos
#include "armor_pos.hpp"
#include "detector.hpp"
#include "Trans.hpp"
#include "image.hpp"

#define DESTROY -1
#define LOST 0
#define PREDICT 1

#define MIN_IOU 0.3

#define DESTORY_FPS 10
struct DetectResult
{
    cv::Point3f ypr;
    cv::Point3f t;
    float dist;
    DetectResult()
    {
        ypr = cv::Point3f(0, 0, 0);
        t = cv::Point3f(0, 0, 0);
        dist = 0;
    }
    DetectResult(cv::Point3f &&r, cv::Point3f &&t)
    {
        this->ypr = r;
        this->t = t;
        this->dist = getDist(t);
    }
    DetectResult(const DetectResult &r)
    {
        this->ypr = r.ypr;
        this->t = r.t;
        this->dist = r.dist;
    }
    static float getDist(const cv::Point3f &t)
    {
        return std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
    }
};

class tracker
{
public:
    int predict(const Image &frame, const std::vector<bbox_t> &in, DetectResult &out);

    tracker(bool enemy, int robot_id, int light_threshold, int dark_threshold, int _destory_limit = 5);

    bbox_t last_bbox;

    cv::Point3f speed;

    std::vector<bbox_t> armors_bbox;

private:
    bool no_last;
    bool enemy;
    int img_cols, img_rows;
    cv::KalmanFilter KF;
    int offline_counter;
    int light_threshold;
    int dark_threshold;
    const int destory_limit;

    float shoot_delay;

    std::vector<cv::Point3f> armor_corner;
    cv::Mat inner;
    cv::Mat dist;

    Eigen::Vector3f last;
    float timeStamp;

    Trans A2world, cam2A, A2pit, A2yaw;

    bool IOUFilter(const std::vector<bbox_t> &in);
    bool NearstArmor(void);
    void GetArmorPos(const std::pair<Vector2f, Vector2f> &lb_v, DetectResult &res);
    void GetConernPoints(const std::pair<Vector2f, Vector2f> &b, std::vector<cv::Point2f> &img_tar);

    cv::Rect getLegalRect(const cv::Mat &frame, const bbox_t &out);

    bool GetSpeed();

    static int GetIOU(const bbox_t &a, const bbox_t &b);
    static int GetDistSq(const bbox_t &box, const cv::Point2f &center);
    static int GetDist(const bbox_t &box, const cv::Point2f &center);
    static std::pair<float, float> CalZoffset(const Eigen::Vector3f &pose_world, float v_shoot);
};

#endif
