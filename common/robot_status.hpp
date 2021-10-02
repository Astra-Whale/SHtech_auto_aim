//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Class for robot status
//

#ifndef COMMON_ROBOT_H
#define COMMON_ROBOT_H
#include <cstdint>
#include <array>

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
    float robot_speed_mps = 28.;
    uint16_t cruise_speed;
    std::array<uint8_t, 6> enemy;             // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
                                              // int(真实血量 / 10)
    GameState game_state = GameState::COMMON; // 是否设计远处
    uint8_t lrc = 0;
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
#endif //COMMON_ROBOT_H
