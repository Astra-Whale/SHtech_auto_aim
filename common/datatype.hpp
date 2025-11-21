//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Modified by Haoran Jiang on 21-10-21: Refact framework: Modified file structure and components of RobotStatus
// Classes of Common Data Type
//

#ifndef COMMON_ROBOT_H
#define COMMON_ROBOT_H

#include "common.hpp"

// packages
#include <cstdint>
#include <array>
#include <chrono>
#include <Eigen/Dense>

enum class EnemyColor : uint8_t
{
    // 敌方颜色
    RED = 0,
    BLUE = 1,
    GRAY = 2,
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

enum class SubModuleResult : uint8_t
{
    // 子模块处理结果
    SUCCESS,    // 成功，数据有效
    SKIP,       // 模块处理被跳过
    FAILURE,    // 失败
    NOTYET,     // 尚未处理
};

enum class SubModuleName : uint8_t
{
    // 子模块名称
    ENTRYSTAGE,
    SENSOR,
    DETECT,
    LINEARPREDICTOR,
    PLANNING,

    COUNT, // 仅用于计数子模块数量
};

// 根据 SubModuleName 自动生成子模块数量常量
constexpr size_t SUBMODULE_COUNT = static_cast<size_t>(SubModuleName::COUNT);

// 获取子模块名称字符串
inline const char* getSubModuleName(SubModuleName module) {
    switch (module) {
        case SubModuleName::ENTRYSTAGE: return "Entry";
        case SubModuleName::SENSOR: return "Sensor";
        case SubModuleName::DETECT: return "Detect";
        case SubModuleName::LINEARPREDICTOR: return "Predict";
        case SubModuleName::PLANNING: return "Planning";
        default: return "Unknown";
    }
}

struct RobotStatus
{
    float robot_speed_mps = 28.0f;
    uint16_t enemy[6];                        // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
    GameState game_state = GameState::COMMON; // 是否设计远处
    EnemyColor enemy_color = EnemyColor::RED;
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
};

struct RobotCommand
{
    float distance;
    float yaw_angle;
    float yaw_speed;
    float pitch_angle;
    float pitch_speed;
    int target_id;
    ShootMode shoot_mode;
};


// 射击指令线性插值，对target_id和shoot_mode采纳时间上更接近的指令
inline RobotCommand command_linear_interpolation(const RobotCommand& cmd1, const RobotCommand& cmd2, float cmdTwoWeight){
    assert(cmdTwoWeight >= 0.0f && cmdTwoWeight <= 1.0f&&"command_linear_interpolation: cmdTwoWeight out of range [0,1]");
    if(cmdTwoWeight < 0.5f)
        return RobotCommand{
            cmd1.distance * (1-cmdTwoWeight) + cmd2.distance * cmdTwoWeight,
            cmd1.yaw_angle * (1-cmdTwoWeight) + cmd2.yaw_angle * cmdTwoWeight,
            cmd1.yaw_speed * (1-cmdTwoWeight) + cmd2.yaw_speed * cmdTwoWeight,
            cmd1.pitch_angle * (1-cmdTwoWeight) + cmd2.pitch_angle * cmdTwoWeight,   
            cmd1.pitch_speed * (1-cmdTwoWeight) + cmd2.pitch_speed * cmdTwoWeight,
            cmd1.target_id,
            cmd1.shoot_mode
        };
    return RobotCommand{
            cmd1.distance * (1-cmdTwoWeight) + cmd2.distance * cmdTwoWeight,
            cmd1.yaw_angle * (1-cmdTwoWeight) + cmd2.yaw_angle * cmdTwoWeight,
            cmd1.yaw_speed * (1-cmdTwoWeight) + cmd2.yaw_speed * cmdTwoWeight,
            cmd1.pitch_angle * (1-cmdTwoWeight) + cmd2.pitch_angle * cmdTwoWeight,   
            cmd1.pitch_speed * (1-cmdTwoWeight) + cmd2.pitch_speed * cmdTwoWeight,
            cmd2.target_id,
            cmd2.shoot_mode
        };
}


struct bbox_t
{
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base

    bool operator==(const bbox_t &a) const
    {
        return pts[0] == a.pts[0] && pts[1] == a.pts[1] && pts[2] == a.pts[2] && pts[3] == a.pts[3];
    }
    bool operator!=(const bbox_t &a) const
    {
        return !(*this == a);
    }
    
    bool operator < (const bbox_t &a) const
    {
        return confidence < a.confidence;
    }
    
    bool operator > (const bbox_t &a) const
    {
        return confidence > a.confidence;
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
            Eigen::AngleAxisf(pitch / 180 * M_PI, Eigen::Vector3f::UnitX());
        return q;
    }
    void toQuaternion(Eigen::Quaternionf &q)
    {
        q = Eigen::AngleAxisf(yaw / 180 * M_PI, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch / 180 * M_PI, Eigen::Vector3f::UnitX());
    }
};

/**
 * @brief   标准报文类
 */
struct ThreadDataPack
{
    cv::Mat frame;              /*!< 读取到的原始图像 */
    std::vector<bbox_t> bboxes; /*!< 检测到的bounding boxes */
    std::chrono::high_resolution_clock::time_point time{}; /*!< 图像时间戳 */

    std::array<SubModuleResult, SUBMODULE_COUNT> submodule_results; /*!< 各子模块处理结果 */

    // 为entrystage子模块添加的处理开始时间戳。当我们开始显示处理时间，包已经进入了下一个流水线阶段，因此它的开始时间已经被更新为下一个流水线阶段的时间了
    // 所以我们要单独记录entrystage的开始时间，并在每次entrystage打印完处理时间后再更新
    std::chrono::steady_clock::time_point pipeline_enter_time{}; 
    std::array<std::pair<std::chrono::steady_clock::time_point, std::chrono::steady_clock::time_point>, SUBMODULE_COUNT> submodule_timestamps{};

    Eigen::Matrix<double, 6, 1> target_state; /*!< 目标状态量 */

    RobotStatus robotstatus;    /*!< 上行机器人状态 */
    Attitude attitude;          /*!< 上行位姿数据 */
    RobotCommand robotcommand;
    int index;                  /*!< 报文序号 */

};



#endif // COMMON_ROBOT_H
