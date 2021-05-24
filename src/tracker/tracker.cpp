#include "tracker.hpp"
#include "log.hpp"
#include "comm.h"

tracker::tracker(bool enemy,
                 int robot_id,
                 int light_threshold,
                 int dark_threshold, KF_detetc_param_t kf_param,
                 int _destory_limit) : destory_limit(_destory_limit),
                                       A2world("A", "world"),
                                       cam2A("cam", "A"),
                                       A2pit("A", "Pit"),
                                       A2yaw("A", "Yaw")
{
    this->enemy = enemy;
    this->light_threshold = light_threshold;
    this->dark_threshold = dark_threshold;

    no_last = true;
    last = Eigen::Vector3f::Zero();

    Eigen::Matrix2f F;
    F << 1, 1, 0, 1;

    const Eigen::RowVector2f H(1, 0);
    float observe_noise = std::get<0>(kf_param);
    float pos_noise = std::get<1>(kf_param);
    float spd_noise = std::get<2>(kf_param);

    x.state_trans_matrix = F;
    x.observe_matrix = H;
    x.cov_matrix_pre = x.cov_matrix_last = Eigen::Matrix2f::Zero();
    x.kalman_gain = Eigen::Vector2f::Zero();
    x.observe_noise << observe_noise;
    x.process_noise << pos_noise, 0, 0, spd_noise;

    y.state_trans_matrix = F;
    y.observe_matrix = H;
    y.cov_matrix_pre = x.cov_matrix_last = Eigen::Matrix2f::Zero();
    y.kalman_gain = Eigen::Vector2f::Zero();
    y.observe_noise << observe_noise;
    y.process_noise << pos_noise, 0, 0, spd_noise;

    z.state_trans_matrix = F;
    z.observe_matrix = H;
    z.cov_matrix_pre = x.cov_matrix_last = Eigen::Matrix2f::Zero();
    z.kalman_gain = Eigen::Vector2f::Zero();
    z.observe_noise << observe_noise;
    z.process_noise << pos_noise, 0, 0, spd_noise;

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
        shoot_delay = 0.2;
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
    track_frame = 0;
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

int tracker::predict(const Image &frame, std::vector<bbox_t> &in, DetectResult &out)
{
    int state = LOST;
    DetectResult now;

    if (track_frame++ > MAX_FRAME)
    {
        return DESTROY;
    }

    do
    {
        if (!IOUFilter(in))
        {
            LOGM_F("empty:(1,%d)", frame.index);
            break;
        }
        if (!NearstArmor())
        {
            LOGM_F("iou:(1,%d)", frame.index);
            break;
        }

        cv::Mat armor = cv::Mat(frame.frame, getLegalRect(frame.frame, last_bbox));

        std::pair<Vector2f, Vector2f> lb_v;

        if (!findLightBlobs(armor, lb_v, enemy, light_threshold, dark_threshold))
        {
            LOGM_F("light_bin:(1,%d)", frame.index);
            break;
        }

        GetArmorPos(lb_v, now);

        if (now.dist > 1000 || now.dist < 10)
        {
            LOGM_F("wrong_dist:(1,%d)", frame.index);
            break;
        }

        //LOGM_S("[DETECT] (x,y,z): %.1f, %.1f, %.1f dist: %.1f ", now.t.x, now.t.y, now.t.z, now.dist);
        if (gim_state.shoot_speed == 0)
        {
            gim_state.shoot_speed = 10;
        }
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

        LOGM_F("now_x:(%f,%d)", now.t.x, 1);
        LOGM_F("now_y:(%f,%d)", now.t.y, 1);
        LOGM_F("now_z:(%f,%d)", now.t.z, 1);
        LOGM_F("world_x:(%f,%d)", armor_world(0), frame.index);
        LOGM_F("world_y:(%f,%d)", armor_world(1), frame.index);
        LOGM_F("world_z:(%f,%d)", armor_world(2), frame.index);

        if (last != Eigen::Vector3f::Zero())
        {
            Eigen::Vector3f dist = armor_world - last;
            printf("now:(%.2f,%.2f,%.2f)\n", armor_world(0), armor_world(1), armor_world(2));
            printf("last:(%.2f,%.2f,%.2f)\n", last(0), last(1), last(2));
            printf("dist:(%.2f,%.2f,%.2f), offline:%d\n", dist(0), dist(1), dist(2), offline_counter);
            LOGM_F("dist_x:(%f,%d)", dist(0), frame.index);
            LOGM_F("dist_y:(%f,%d)", dist(1), frame.index);
            LOGM_F("dist_z:(%f,%d)", dist(2), frame.index);

            if (dist.norm() > MAX_DIFF)
            {
                LOGM_F("far_data:(1,%d)", frame.index);
                //armor_world = last;
                //break;
            }
        }
        else
        {
            x.x_last = Eigen::Vector2f(armor_world(0), 0);
            y.x_last = Eigen::Vector2f(armor_world(1), 0);
            z.x_last = Eigen::Vector2f(armor_world(2), 0);
            LOGM_F("no_last:(1,%d)", frame.index);
        }

        float time_interval = (systime.getTime() - timeStamp) / 1000;
        x.state_trans_matrix(2) = time_interval;
        y.state_trans_matrix(2) = time_interval;
        z.state_trans_matrix(2) = time_interval;

        Eigen::Vector2f x_kf = x.update(Eigen::Matrix<float, 1, 1>(armor_world(0)), Eigen::Matrix<float, 0, 1>(0));
        Eigen::Vector2f y_kf = y.update(Eigen::Matrix<float, 1, 1>(armor_world(1)), Eigen::Matrix<float, 0, 1>(0));
        Eigen::Vector2f z_kf = z.update(Eigen::Matrix<float, 1, 1>(armor_world(2)), Eigen::Matrix<float, 0, 1>(0));
        LOGM_F("x_kf:(%f,%d)", x_kf(0), frame.index);
        LOGM_F("y_kf:(%f,%d)", y_kf(0), frame.index);
        LOGM_F("z_kf:(%f,%d)", z_kf(0), frame.index);
        LOGM_S("pos_kf:(%.2f,%.2f,%.2f,%d)", x_kf(0), y_kf(0), y_kf(0), frame.index);

        last = Eigen::Vector3f(x_kf(0), y_kf(0), z_kf(0));
        auto speed_tmp = Eigen::Vector3f(x_kf(1), y_kf(1), z_kf(1));

        if (speed_tmp.norm() > 800)
        {
            LOGM_F("huge_speed:(1,%d)", frame.index);
            //break;
        }
        else
        {
            speed = speed_tmp;
        }

        LOGM_F("speed_x:(%f,%d)", speed(0), frame.index);
        LOGM_F("speed_y:(%f,%d)", speed(1), frame.index);
        LOGM_F("speed_z:(%f,%d)", speed(2), frame.index);
        std::pair<float, float> z_offset_and_t = CalZoffset(armor_world / 100, gim_state.shoot_speed);
        float z_offset = z_offset_and_t.first;
        float t = z_offset_and_t.second;

        //speed = Eigen::Vector3f(0, 0, 0); //cv::Point3f(0, 0, 0);
        armor_world = (t + shoot_delay) * speed + armor_world;
        last = armor_world;

        z_offset_and_t = CalZoffset(armor_world / 100, gim_state.shoot_speed);

        z_offset = z_offset_and_t.first * 100;
        t = z_offset_and_t.second;
        timeStamp = systime.getTime();

        //Here is the log of z_offset;
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

        state = PREDICT;
    } while (0);
    if (state != PREDICT)
    {
        no_last = true;
        offline_counter++;
        state = LOST;
        if (offline_counter > destory_limit)
        {
            state = DESTROY;
        }
    }
    armors_bbox.clear();

    LOGM_F("offline:(%d,%d)", offline_counter, frame.index);

    return state;
}

bool tracker::IOUFilter(std::vector<bbox_t> &in)
{
    if (in.empty())
    {
        no_last = true;
        return false;
    }
    std::sort(in.begin(), in.end(), [](const bbox_t &a, const bbox_t &b)
              { return a.prob > b.prob; });
    for (unsigned int i = 0; i < in.size(); i++)
    {
        bool flag_big_IOU = false;
        for (unsigned int j = i + 1; j < in.size(); j++)
        {
            if (GetIOU(in[i], in[j]) > 0.7)
            {
                flag_big_IOU = true;
                break;
            }
        }
        if (!flag_big_IOU)
        {
            armors_bbox.push_back(in[i]);
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
                  { return GetDistSq(a, center) < GetDistSq(b, center); });
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
    auto bbox = cv::Rect(out.x - out.w * 0.05, out.y - out.h * 0.05,
                         out.w * 1.1, out.h * 1.1);
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
