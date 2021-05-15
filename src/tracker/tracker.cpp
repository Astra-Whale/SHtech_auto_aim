#include "tracker.hpp"
#include "log.hpp"
#include "comm.h"

tracker::tracker(bool enemy, int robot_id, int light_threshold, int dark_threshold, int _destory_limit) : destory_limit(_destory_limit)
{
    this->enemy = enemy;
    this->light_threshold = light_threshold;
    this->dark_threshold = dark_threshold;
    float trans[6][6] = {0};
    trans[0][3] = trans[1][4] = trans[2][5] = 1;
    for (int i = 0; i < 6; i++)
    {
        trans[i][i] = 1;
    }

    no_last = true;

    KF.init(6, 3, 0);
    cv::Mat(6, 6, CV_32F, trans).copyTo(KF.transitionMatrix);
    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(80));    //系统噪声方差矩阵Q
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1)); //测量噪声方差矩阵R
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(300));      //后验错误估计协方差矩阵P

    //KF.statePost = (cv::setIdentity<float>(6, 1) << target.t.x, target.t.y, target.t.z, 0, 0, 0);
    offline_counter = 0;

    armor_corner.emplace_back(cv::Point3f(-7, 2.75, 0));
    armor_corner.emplace_back(cv::Point3f(-7, -2.75, 0));
    armor_corner.emplace_back(cv::Point3f(7, 2.75, 0));
    armor_corner.emplace_back(cv::Point3f(7, -2.75, 0));

    shoot_delay = 0; //s^-1

    float b[3][3] = {0};
    if (robot_id == 1)
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }
    else if (robot_id == 2)
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }
    else if (robot_id == 3)
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }
    else if (robot_id == 4)
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }
    else if (robot_id == 5)
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }

    cv::Mat(3, 3, CV_32F, b).copyTo(inner);
    dist = cv::Mat::zeros(5, 1, CV_32F);

    speed = cv::Point3f(0, 0, 0);

    //std::cout << "Transition Matrix: " << std::endl
    //          << KF.transitionMatrix << std::endl;
    //std::cout << "Measurement Matrix: " << std::endl
    //          << KF.measurementMatrix << std::endl;
    //std::cout<<"statePost: "<<KF.statePost<<std::endl;
}

cv::Point3f tracker::CalYPT(const cv::Point3f &pose_world, float v_shoot)
{
    float g = 9.8;
    float delta = pow((g * pose_world.z - v_shoot * v_shoot), 2) - g * g * (pose_world.x * pose_world.x + pose_world.y * pose_world.y + pose_world.z * pose_world.z);
    if (delta < 0)
    {
        return cv::Point3f(0, 0, 0);
    }
    float t_2 = v_shoot * v_shoot - g * pose_world.z - sqrt(delta);
    t_2 = 2 * t_2 / g / g;
    float t = sqrt(t_2);

    float yaw = atan2(pose_world.x, pose_world.y);
    float pit = -1.0f * asin(pose_world.z + 0.5 * g * t_2 / v_shoot / t);
    return cv::Point3f(yaw, pit, t);
}

int tracker::predict(const cv::Mat &frame, const std::vector<bbox_t> &in, DetectResult &out)
{

    if (IOUFilter(in) == false)
    {
        offline_counter++;
        if (offline_counter > destory_limit)
        {
            return DESTROY;
        }
        return LOST;
    }
    if (NearstArmor() == false)
    {
        offline_counter++;
        if (offline_counter > destory_limit)
        {
            return DESTROY;
        }
        return LOST;
    }

    cv::Mat armor = cv::Mat(frame, getLegalRect(frame, last_bbox));

    std::pair<Vector2f, Vector2f> lb_v;

    if (findLightBlobs(armor, lb_v, enemy, light_threshold, dark_threshold) == false)
    {
        offline_counter++;
        if (offline_counter > destory_limit)
        {
            return DESTROY;
        }
        return LOST;
    }

    DetectResult now;

    GetArmorPos(lb_v, now);

    if (now.dist > 500 || now.dist < 10)
    {
        offline_counter++;
        if (offline_counter > destory_limit)
        {
            return DESTROY;
        }
        return LOST;
    }

    //LOGM_S("[DETECT] (x,y,z): %.1f, %.1f, %.1f dist: %.1f ", now.t.x, now.t.y, now.t.z, now.dist);

    float dist_ny = sqrt(now.t.x * now.t.x + now.t.z * now.t.z);
    out.ypr.y = atan2(now.t.y, dist_ny);
    out.ypr.x = atan2(now.t.x, now.t.z);
    out.ypr.z = 0;
    out.ypr = out.ypr / 3.1415 * 180;
    //LOGM_S("[REF] (x,y,z): %.1f, %.1f, %.1f dist: %.1f ", out.ypr.x, out.ypr.y, out.ypr.z, now.dist);

    cv::Mat xyz_cam = (cv::Mat_<float>(3, 1) << gim_state.curr_pitch, gim_state.curr_yaw, 0);
    cv::Mat cam2Zworld;
    cv::Rodrigues(xyz_cam, cam2Zworld);
    cv::Mat armor_t_cam = (cv::Mat_<float>(3, 1) << now.t.x, now.t.y, now.t.z);
    cv::Mat armor_t_Zworld = cam2Zworld * armor_t_cam; // z front

    cv::Point3f armor_t_world = cv::Point3f(armor_t_Zworld.at<float>(0, 0), armor_t_Zworld.at<float>(2, 0), -armor_t_Zworld.at<float>(1, 0));

    cv::Point3f speed;
    if (no_last)
    {
        speed = cv::Point3f(0, 0, 0);
    }
    else
    {
        float now = systime.getTime();
        speed = (armor_t_world - last) / (now - timeStamp) * 0.3;
        last = armor_t_world;
        timeStamp = now;
    }

    cv::Point3f predict_ypt = CalYPT(armor_t_world / 100, gim_state.shoot_speed);

    speed = cv::Point3f(0, 0, 0);
    armor_t_world = (predict_ypt.z + shoot_delay) * speed + armor_t_world;

    predict_ypt = CalYPT(armor_t_world / 100, gim_state.shoot_speed);

    if (gim_state.shoot_speed < 3)
    {
    }
    else
    {
        out.ypr.x = predict_ypt.x / 3.1415 * 180;
        out.ypr.y = predict_ypt.y / 3.1415 * 180;
    }

    offline_counter = 0;
    return PREDICT;
}

bool tracker::IOUFilter(const std::vector<bbox_t> &in)
{
    if (in.empty())
    {
        no_last = true;
        return false;
    }
    std::sort(armors_bbox.begin(), armors_bbox.end(), [](const bbox_t &a, const bbox_t &b) { return a.prob > b.prob; });
    for (bbox_t i : in)
    {
        bool flag_big_IOU = false;
        for (bbox_t &j : armors_bbox)
        {
            if (GetIOU(i, j) > 0.6)
            {
                flag_big_IOU = true;
                break;
            }
        }
        if (!flag_big_IOU)
        {
            armors_bbox.push_back(i);
        }
    }
    return true;
}

bool tracker::NearstArmor(void)
{
    if (no_last)
    {
        cv::Point2f center = cv::Point2f(img_cols / 2, img_rows / 2);
        std::sort(armors_bbox.begin(),
                  armors_bbox.end(),
                  [&center](const bbox_t &a, const bbox_t &b) { return GetDistSq(a, center) > GetDistSq(b, center); });
        last_bbox = armors_bbox[0];
    }
    else
    {
        std::sort(armors_bbox.begin(),
                  armors_bbox.end(),
                  [&](const bbox_t &a, const bbox_t &b) { return GetIOU(a, last_bbox) < GetIOU(b, last_bbox); });
        if (GetIOU(armors_bbox[0], last_bbox) < MIN_IOU)
        {
            last_bbox = armors_bbox[0];
        }
        else
        {
            no_last = true;
            return false;
        }
    }
    no_last = false;
    return true;
}

void tracker::GetArmorPos(const std::pair<Vector2f, Vector2f> &lb_v, DetectResult &res)
{
    std::vector<cv::Point2f> img_tar;
    cv::Mat R, t;

    cv::Point2f center = cv::Point2f(last_bbox.x, last_bbox.y);
    GetConernPoints(lb_v, img_tar);
    for (auto &i : img_tar)
    {
        i += center;
    }

    solvePnP(armor_corner, img_tar, inner, dist, R, t);

    R = -R / 3.1415 * 180;
    //std::cout << R << R.type()<< t<<t.type() << std::endl;
    res = DetectResult(cv::Point3f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2)), cv::Point3f(t.at<double>(0, 0), t.at<double>(0, 1), t.at<double>(0, 2)));
}

void tracker::GetConernPoints(const std::pair<Vector2f, Vector2f> &b, std::vector<cv::Point2f> &img_tar)
{
    Vector2f v1 = b.first;
    Vector2f v2 = b.second;
    if (v1.dot(v2) < 0)
    {
        v2.flip();
    }
    cv::Point2f v_mid = v2.v + v1.v;
    v_mid = cv::Point2f(v_mid.y, -v_mid.x); // vertical mid vector
    cv::Point2f v_mid_rename = v2.end + v2.begin - v1.end - v1.begin;

    Vector2f v_left, v_right;
    if (v_mid.dot(v_mid_rename) > 0)
    {
        v_left = v1;
        v_right = v2;
    }
    else
    {
        v_left = v2;
        v_right = v1;
    }
    img_tar.push_back(v_left.begin);
    img_tar.push_back(v_left.end);
    img_tar.push_back(v_right.begin);
    img_tar.push_back(v_right.end);

    //cv::solvePnP(armor_corner,img_tar,);
}

int tracker::GetIOU(const bbox_t &a, const bbox_t &b)
{
    int in_w = std::min(a.x + a.w, b.x + b.w) - std::max(a.x, b.x);
    int in_h = std::min(a.y + a.h, b.y + b.h) - std::max(a.y, b.y);
    if (in_w < 0 || in_h < 0)
    {
        return 0;
    }
    else
    {
        return in_w * in_h / (a.w * a.h + b.w * b.h - in_w * in_h);
    }
}

int tracker::GetDistSq(const bbox_t &box, const cv::Point2f &center)
{
    return (box.x + box.w / 2 - center.x) * (box.x + box.w / 2 - center.x) + (box.y + box.h / 2 - center.y) * (box.x + box.h / 2 - center.y);
}

cv::Rect tracker::getLegalRect(const cv::Mat &frame, const bbox_t &out)
{
    auto bbox = cv::Rect(out.x - out.w * 0.10, out.y - out.h * 0.10,
                         out.w * 1.2, out.h * 1.2);
    if (bbox.x < 0)
    {
        bbox.x = 0;
    }
    if (bbox.y < 0)
    {
        bbox.y = 0;
    }
    if (bbox.x + bbox.width > frame.cols)
    {
        bbox.width = frame.cols - bbox.x;
    }
    if (bbox.y + bbox.height > frame.rows)
    {
        bbox.height = frame.rows - bbox.y;
    }
    return bbox;
}
