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
#include <stdint.h>
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

struct RobotStatus
{
    ProgramMode program_mode = ProgramMode::AUTO_AIM;
    float robot_speed_mps = 28.0f;
    uint16_t enemy[6];                        // 敌方哨兵0、英雄1、工程2、步兵3、步兵4、步兵5
    GameState game_state = GameState::COMMON; // 是否设计远处
    EnemyColor enemy_color = EnemyColor::RED;
};

struct RobotCommand
{
    float distance;
    float yaw_angle;
    float yaw_speed;
    float pitch_angle;
    float pitch_speed;
    ShootMode shoot_mode;
    int target_id;
};

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

//装甲板的位置和朝向
class ArmorPlacement {
public:
    Eigen::Vector3d location;//位置
    Eigen::Vector3d normal;//法向量，指向装甲板背面。一般地：指向车体内部。
    int armor_number;
    ArmorPlacement(Eigen::Vector3d location, Eigen::Vector3d normal,int armor_number)
       : location(location), normal(normal),armor_number(armor_number) {}
    double x() const { return location(0); }
    double y() const { return location(1); }
    double z() const { return location(2); }
    ~ArmorPlacement() = default;
    virtual double get_angle() const = 0;
};

//世界坐标系下装甲板的位置和朝向
class ArmorPlacementPW : public ArmorPlacement {
public:
    ArmorPlacementPW(Eigen::Vector3d location, Eigen::Vector3d normal,int armor_number)
       : ArmorPlacement(location, normal, armor_number) {}
    // 获取世界坐标系下装甲板方向在xy平面上（地面上）的投影与x正半轴的夹角，逆时针为正。从0°到360°
    // 即：完全与一般的平面坐标系里角度的定义一致
    double get_angle() const override
    {
        Eigen::Vector3d x_axis(1, 0, 0);  // 改用x轴作为参考轴
        // 投影到xy平面
        Eigen::Vector3d normal_xy(normal.x(), normal.y(), 0);
        normal_xy.normalize();
        
        double cos_angle = normal_xy.dot(x_axis);
        double angle = std::acos(cos_angle);
        
        // 判断逆时针/顺时针
        if(normal_xy.y() < 0) {  // 如果y分量为负，说明在下半平面
            angle = 2 * M_PI - angle;
        }
        
        return angle * 180.0 / M_PI;
    }

    void printInfo() const {
        std::cout << "\n=== Armor Placement Information ===\n";
        
        // 装甲板编号
        std::cout << "Armor Number: " << armor_number << "\n";
        
        // 位置信息 - 保留3位小数
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Position (x, y, z): ("
                << location.x() << ", "
                << location.y() << ", "
                << location.z() << ") m\n";
        
        // 法向量信息
        std::cout << "Normal Vector: ("
                << normal.x() << ", "
                << normal.y() << ", "
                << normal.z() << ")\n";
        
        // 角度信息 - 保留1位小数
        std::cout << std::setprecision(1);
        std::cout << "Angle in XY Plane: " << get_angle() << "°\n";
        
        // 计算并显示与地面的倾角
        double pitch = std::acos(normal.dot(Eigen::Vector3d(0, 0, 1))) * 180.0 / M_PI;
        std::cout << "Pitch Angle: " << pitch << "°\n";
        
        // 计算装甲板在地面上的投影距离
        double ground_distance = std::sqrt(location.x() * location.x() + location.y() * location.y());
        std::cout << std::setprecision(3);
        std::cout << "Ground Projection Distance: " << ground_distance << " m\n";
        
        std::cout << "===================================\n";
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
    int index;                  /*!< 报文序号 */
    cv::Mat frame;              /*!< 读取到的原始图像 */
    std::vector<bbox_t> bboxes; /*!< 检测到的bounding boxes */
    RobotStatus robotstatus;    /*!< 上行机器人状态 */
    Attitude attitude;          /*!< 上行位姿数据 */
    RobotCommand robotcommand;
    std::chrono::high_resolution_clock::time_point time;
};

#endif // COMMON_ROBOT_H
