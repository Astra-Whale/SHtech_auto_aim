//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Modified by Haoran Jiang on 21-10-21: Refact framework: Modified file structure and components of RobotStatus
// Classes of Common Data Type
//

#ifndef COMMON_ROBOT_H
#define COMMON_ROBOT_H


// packages
#include <cstdint>
#include <array>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

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
    MULTI_POLICY_PREDICTOR,
    PLANNER,

    COUNT, // 仅用于计数子模块数量
};

enum class DetectionSource : uint8_t
{
    // 检测来源
    NEURAL_NETWORK = 0, // 神经网络
    TRADITIONAL = 1, // 传统方法
};

// 根据 SubModuleName 自动生成子模块数量常量
constexpr size_t SUBMODULE_COUNT = static_cast<size_t>(SubModuleName::COUNT);

// 获取子模块名称字符串
inline const char* getSubModuleName(SubModuleName module) {
    switch (module) {
        case SubModuleName::ENTRYSTAGE: return "Entry";
        case SubModuleName::SENSOR: return "Sensor";
        case SubModuleName::DETECT: return "Detect";
        case SubModuleName::MULTI_POLICY_PREDICTOR: return "Predict";
        case SubModuleName::PLANNER: return "Planner";
        default: return "Unknown";
    }
}

constexpr float INF_BALL_SPEED = 30.0f; // 步兵弹速默认值 m/s

struct RobotStatus
{
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
    float robot_speed_mps = INF_BALL_SPEED;
    uint16_t enemy[6];                        // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
    GameState game_state = GameState::COMMON; // 是否设计远处
    EnemyColor enemy_color = EnemyColor::RED;
};

struct RobotCommand
{
    float distance;
    float yaw_angle;
    float yaw_speed;
    float yaw_acc;
    float pitch_angle;
    float pitch_speed;
    float pitch_acc;
    int fire_enable; // 0 is disable, 1 is enable, 2 is self-determined
    int target_id; // 1-7: robot, 8: outpost, 9: base, 0: none
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
            cmd1.fire_enable
        };
    return RobotCommand{
            cmd1.distance * (1-cmdTwoWeight) + cmd2.distance * cmdTwoWeight,
            cmd1.yaw_angle * (1-cmdTwoWeight) + cmd2.yaw_angle * cmdTwoWeight,
            cmd1.yaw_speed * (1-cmdTwoWeight) + cmd2.yaw_speed * cmdTwoWeight,
            cmd1.pitch_angle * (1-cmdTwoWeight) + cmd2.pitch_angle * cmdTwoWeight,   
            cmd1.pitch_speed * (1-cmdTwoWeight) + cmd2.pitch_speed * cmdTwoWeight,
            cmd2.target_id,
            cmd2.fire_enable
        };
}


struct bbox_t
{
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    float confidence;
    int color_id; // 0: red, 1: blue, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base
    DetectionSource source;   // 0: neural network, 1: traditional cv

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
    // 1. 默认构造函数：ypr等于0，即水平姿态
    Attitude() : Attitude(0.0f, 0.0f, 0.0f) {}

    Attitude(float yaw, float pitch, float roll) 
        : yaw_(yaw), pitch_(pitch), roll_(roll) 
    {
        const double deg2rad = M_PI / 180.0;

        Eigen::Quaterniond q = 
            Eigen::AngleAxisd(static_cast<double>(yaw)   * deg2rad, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(static_cast<double>(pitch) * deg2rad, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(static_cast<double>(roll)  * deg2rad, Eigen::Vector3d::UnitY());

        // 计算 World -> IMU 旋转
        // R_world2imu_ = R_initial * R_relative.transpose()
        R_world2imu_ = get_initial_transform() * q.toRotationMatrix().transpose();
    }

    float yaw() const { return yaw_; }
    float pitch() const { return pitch_; }
    float roll() const { return roll_; }
    const Eigen::Matrix3d& R_world2imu() const { return R_world2imu_; }

private:
    float yaw_;
    float pitch_;
    float roll_;
    Eigen::Matrix3d R_world2imu_;

    /**
     * @brief 自定义坐标系到PnP坐标系的转换矩阵
     * @details 该矩阵用于将自定义的世界坐标系转换为PnP算法使用的标准坐标系
     *          实现坐标系的翻转和轴交换
     */
    static const Eigen::Matrix3d& get_initial_transform()
    {
        static const Eigen::Matrix3d R = (Eigen::Matrix3d() << 
            -1.,  0.,  0.,
             0.,  0.,  1.,
             0.,  1.,  0. ).finished();
        return R;
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

    std::array<std::pair<std::chrono::steady_clock::time_point, std::chrono::steady_clock::time_point>, SUBMODULE_COUNT> submodule_timestamps{};

    Eigen::Matrix<double, 6, 1> target_state; /*!< 目标状态量 */

    RobotStatus robotstatus;    /*!< 上行机器人状态 */
    Attitude attitude;          /*!< 上行位姿数据 */
    RobotCommand robotcommand;
    int index;                  /*!< 报文序号 */

};



#endif // COMMON_ROBOT_H
