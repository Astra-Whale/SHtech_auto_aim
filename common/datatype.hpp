//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Modified by Haoran Jiang on 21-10-21: Refact framework: Modified file structure and components of RobotStatus
// Classes of Common Data Type
//

#ifndef COMMON_ROBOT_H
#define COMMON_ROBOT_H

#include "common.hpp"

//packages
#include <stdint.h>
#include <array>
#include <chrono>
#include <Eigen/Dense>

enum class EnemyColor : uint8_t
{
    // 敌方颜色
    RED = 0,
    BLUE = 1,
};

enum class ProgramMode : uint8_t
{
    // 视觉模式
    AUTO_AIM = 1,     // 自瞄
    ANTIMISSLE = 2,   // 反导
    SMALL_ENERGY = 4, // 小能量机关
    BIG_ENERGY = 8,   // 大能量机关
};

// 低5位发射标志位，高3位状态标识位
enum class ShootMode : uint8_t
{
    // 射击标志位
    COMMON = 0,      // 普通模式
    DISTANT = 1,     // 远距离击打
    ANTITOP = 2,     // 反陀螺
    SWITCH = 4,      // 快速切换装甲板
    FOLLOW = 8,      // 跟随不发弹
    CRUISE = 16,     // 巡航
    EXIST_HERO = 32, // 英雄存在
};

enum class GameState : uint8_t
{
    // 比赛模式
    SHOOT_NEAR_ONLY = 0, // 仅射击近处
    SHOOT_FAR = 1,       // 允许远处射击
    COMMON = 255,        // 巡航
};

struct RobotStatus
{
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
    float robot_speed_mps = 28.0f;
    uint8_t enemy[6];                         // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
                                              // int(真实血量 / 10)
    GameState game_state = GameState::COMMON; // 是否设计远处
    EnemyColor enemy_color;
} __attribute__((packed));

struct RobotCommand
{
    int distance;
    float yaw_angle;
    float yaw_speed;
    float pitch_angle;
    float pitch_speed;
    uint8_t shoot_mode;
    int target_id;
} __attribute__((packed));

struct bbox_t
{
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base

    bool operator==(const bbox_t &a)
    {
        return pts[0] == a.pts[0] && pts[1] == a.pts[1] && pts[2] == a.pts[2] && pts[3] == a.pts[3];
    }
    bool operator!=(const bbox_t &a)
    {
        return !(*this == a);
    }
};

class Attitude
{
public:
    float yaw, pitch, roll;
    Attitude() : yaw(.0f), pitch(.0f), roll(.0f) {}
    Attitude(float _yaw, float _pitch, float _roll) : yaw(_yaw), pitch(_pitch), roll(_roll) {}
    Eigen::Quaternionf toQuaternion()
    {
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(yaw / 180 * M_PI, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch / 180 * M_PI, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(roll / 180 * M_PI, Eigen::Vector3f::UnitX());
        return q;
    }
    void toQuaternion(Eigen::Quaternionf &q)
    {
        q = Eigen::AngleAxisf(yaw / 180 * M_PI, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch / 180 * M_PI, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(roll / 180 * M_PI, Eigen::Vector3f::UnitX());
    }
};

/**
 * @brief   标准报文类
 */
struct ThreadDataPack
{
    int index;                  /*!< 报文序号 */
    cv::Mat frame;              /*!< 读取到的原始图像 */
    std::vector<bbox_t> bboxes; /*!< 检测到的bounding boxes */
    RobotStatus robotstatus;    /*!< 上行机器人状态 */
    Attitude attitude;          /*!< 上行位姿数据 */
    RobotCommand robotcommand;
    std::chrono::high_resolution_clock::time_point time;
};

#endif //COMMON_ROBOT_H
