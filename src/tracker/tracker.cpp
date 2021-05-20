#include "tracker.hpp"
#include "log.hpp"
#include "comm.h"

tracker::tracker(bool enemy,
                 int robot_id,
                 int light_threshold,
                 int dark_threshold,
                 int _destory_limit) : destory_limit(_destory_limit),
                                       A2world("A", "world"),
                                       cam2A("cam", "A"),
                                       A2pit("A", "Pit"),
                                       A2yaw("A", "Yaw")
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
    if (robot_id == 1) //hero
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};

        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
        cam2A.update_shift(Eigen::Vector3f(-3, -3, 8)); 
        cam2A.update_trans(Eigen::Vector3f(-M_PI_2, 0, -M_PI_2)); // Done
        A2pit.update_shift(Eigen::Vector3f(0, 0, 7));
    }
    else if (robot_id == 2) //doubleshoot
    {
        float a[3][3] = {{1325.6, 0, 630.6053},
                         {0, 1324.5, 527.6321},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
        cam2A.update_shift(Eigen::Vector3f(-1, 5, 6));
        cam2A.update_trans(Eigen::Vector3f(-M_PI_2, 0, -M_PI_2)); // Done
        A2pit.update_shift(Eigen::Vector3f(0, 0, 0));
    }
    else if (robot_id == 3) //infantry
    {
        float a[3][3] = {{1325.412919614640, 0, 641.155914323051},
                         {0, 1324.218656207114, 492.927180047021},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
        cam2A.update_shift(Eigen::Vector3f(-1, 6, 5));
        cam2A.update_trans(Eigen::Vector3f(-M_PI_2, 0, -M_PI_2)); // Done
        A2pit.update_shift(Eigen::Vector3f(0, 0, 0));
    }
    
    else if (robot_id == 4) //aerial
    {
        float a[3][3] = {{873.7503, 0, 638.3636},
                         {0, 872.7696, 514.9894},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }
    else if (robot_id == 5) //sentry
    {
        float a[3][3] = {{1303.581622411977, 0, 646.761077256198},
                         {0, 1306.214349841679, 509.046663300606},
                         {0, 0, 1.000000000000}};
        memcpy(b, a, sizeof(a));
        shoot_delay = 0;
    }

    cv::Mat(3, 3, CV_32F, b).copyTo(inner);
    dist = cv::Mat::zeros(5, 1, CV_32F);

    speed = Eigen::Vector3f(0, 0, 0);

    //std::cout << "Transition Matrix: " << std::endl
    //          << KF.transitionMatrix << std::endl;
    //std::cout << "Measurement Matrix: " << std::endl
    //          << KF.measurementMatrix << std::endl;
    //std::cout<<"statePost: "<<KF.statePost<<std::endl;
}

std::pair<float, float> tracker::CalZoffset(const Eigen::Vector3f &pose_world, float v_shoot)
{
    const float g = 9.8;
    float d_2 = (pose_world(0) * pose_world(0) + pose_world(1) * pose_world(1) + pose_world(2) * pose_world(2));
    float delta = pow((g * pose_world(2) - v_shoot * v_shoot), 2) - g * g * d_2;
    if (delta < 0)
    {
        return std::pair<float, float>(0, std::sqrt(d_2) / 15);
    }
    float t_2 = v_shoot * v_shoot - g * pose_world(2) - sqrt(delta);
    t_2 = t_2 * 2 / g / g;
    return std::pair<float, float>(t_2 * 0.5 * g, std::sqrt(t_2));
}

int tracker::predict(const Image &frame, const std::vector<bbox_t> &in, DetectResult &out)
{
    int state = PREDICT;
    DetectResult now;

    do
    {
        if (!IOUFilter(in))
        {
            offline_counter++;
            if (offline_counter > destory_limit)
            {
                state = DESTROY;
                break;
            }
            state = LOST;
        }
        if (!NearstArmor())
        {
            offline_counter++;
            if (offline_counter > destory_limit)
            {
                state = DESTROY;
            }
            state = LOST;
        }

        cv::Mat armor = cv::Mat(frame.frame, getLegalRect(frame.frame, last_bbox));

        std::pair<Vector2f, Vector2f> lb_v;

        if (!findLightBlobs(armor, lb_v, enemy, light_threshold, dark_threshold))
        {
            offline_counter++;
            if (offline_counter > destory_limit)
            {
                state = DESTROY;
            }
            state = LOST;
        }

        GetArmorPos(lb_v, now);

        if (now.dist > 1000 || now.dist < 10)
        {
            offline_counter++;
            if (offline_counter > destory_limit)
            {
                state = DESTROY;
            }
            state = LOST;
        }
    } while (0);
    if (state == LOST || state == DESTROY)
    {
        return state;
    }

    //LOGM_S("[DETECT] (x,y,z): %.1f, %.1f, %.1f dist: %.1f ", now.t.x, now.t.y, now.t.z, now.dist);
    gim_state.shoot_speed = 15;
    A2world.update_trans(Eigen::Vector3f(gim_state.curr_yaw / 180 * M_PI, gim_state.curr_pitch / 180 * M_PI, 0));

    float dist_ny = sqrt(now.t.x * now.t.x + now.t.z * now.t.z);
    out.ypr.y = atan2(now.t.y, dist_ny);
    out.ypr.x = atan2(now.t.x, now.t.z);
    out.ypr.z = 0;
    out.ypr = out.ypr / 3.1415 * 180;
    //LOGM_S("[REF] (x,y,z): %.1f, %.1f, %.1f dist: %.1f ", out.ypr.x, out.ypr.y, out.ypr.z, now.dist);

    Eigen::Vector3f armor_cam = Eigen::Vector3f(now.t.x, now.t.y, now.t.z);
    Eigen::Vector3f armor_A = cam2A.transform(armor_cam);
    Eigen::Vector3f armor_world = A2world.transform(armor_A);

    if (!no_last && last != Eigen::Vector3f::Zero())
    {
        speed = speed * 0.7 + (armor_world - last)*(systime.getTime()-timeStamp) * (1 - 0.7);
        LOGM_S("Speed:(%.2f,%.2f,%.2f)",speed(0),speed(1),speed(2));
        last = armor_world;
        timeStamp = systime.getTime();
    }
    std::pair<float, float> z_offset_and_t = CalZoffset(armor_world / 100, gim_state.shoot_speed);
    float z_offset = z_offset_and_t.first;
    float t = z_offset_and_t.second;

    //speed = Eigen::Vector3f(0, 0, 0); //cv::Point3f(0, 0, 0);
    armor_world = (t + shoot_delay) * speed + armor_world;

    z_offset_and_t = CalZoffset(armor_world / 100, gim_state.shoot_speed);

    z_offset = z_offset_and_t.first * 100;
    t = z_offset_and_t.second;

    //Here is the log of z_offset;
    if(z_offset == 0)
    {
        printf("damn!!!!! There may have something wrong!!!");
    }
    if(z_offset > 100)
    {
        printf("Oops!!!!!! What's wrong???????????????????");
    }
    LOGM_F("[SEND] z_offset:(%f,%d)", z_offset, 1);

    armor_world(2) = armor_world(2) + z_offset;
    Eigen::Vector3f armor_with_offset_A = A2world.transform_inv(armor_world);

    out.ypr.x = atan2(armor_with_offset_A(1), armor_with_offset_A(0));
    out.ypr.y = atan2(armor_with_offset_A(2), std::sqrt(std::pow(armor_with_offset_A(0), 2) + std::pow(armor_with_offset_A(1), 2)));
    out.ypr.z = t;
    out.ypr.x *= -1.5;
    out.ypr.y *= -1.5;
    out.ypr.x += gim_state.curr_yaw;
    out.ypr.y += gim_state.curr_pitch;
    out.ypr.x = out.ypr.x / 3.1415 * 180;
    out.ypr.y = out.ypr.y / 3.1415 * 180;
    //LOGM_S("IMU_Yaw: %.2f IMU_Pit: %.2f Yaw: %.2f Pitch: %.2f", gim_state.curr_yaw / 3.1415 * 180, gim_state.curr_pitch / 3.1415 * 180, out.ypr.x, out.ypr.y);

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
    std::sort(armors_bbox.begin(), armors_bbox.end(), [](const bbox_t &a, const bbox_t &b)
              { return a.prob > b.prob; });
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
                  [&center](const bbox_t &a, const bbox_t &b)
                  { return GetDistSq(a, center) > GetDistSq(b, center); });
        last_bbox = armors_bbox[0];
    }
    else
    {
        std::sort(armors_bbox.begin(),
                  armors_bbox.end(),
                  [&](const bbox_t &a, const bbox_t &b)
                  { return GetIOU(a, last_bbox) < GetIOU(b, last_bbox); });
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
