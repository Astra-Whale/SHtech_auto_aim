# Auto-Aim

面向 RoboMaster 竞赛的视觉自瞄参考实现。项目以 RM2026 AX650 方案为主线，展示从图像采集、装甲板检测、位姿解算、目标跟踪到云台规划的完整处理链。

本仓库用于说明算法和实现关系。它不是通用自瞄框架，也不承诺覆盖全部硬件、推理后端或实机工况。

## 阅读入口

推荐按以下顺序阅读：

1. 先阅读项目定位和系统流程
2. 再阅读 `main.cpp`，了解对象创建和线程编排
3. 按数据流阅读 `sensor`、`detect`、`predict`、`planner`
4. 对照 `launch.cfg`、相机参数和 Planner 参数
5. 最后查看对应平台的构建条件和已知限制

算法扫盲材料见[飞书文档《只做对的：从0到旋转平移靶80%命中率的自瞄指南》](https://fcn47qghdcqf.feishu.cn/wiki/Hcw1wxTMZicx0xkinuQcKHetn5d?from=from_copylink)。项目演示见[RoboMaster 公开文章](https://bbs.robomaster.com/article/1883871?source=8)。文档讲解通用问题和本项目采用的方案，仓库 README 只保留实现信息。

## 主流程

```text
EntryStage
    ↓
Sensor → Preprocess → Detect
                         ↓
              CornerRefine → Predictor → Planner
                                             ↓
                                         TimedSerial
```

- `EntryStage`：维护数据包、帧号和阶段计时
- `Sensor`：读取海康相机或视频文件，同时获取姿态和机器人状态
- `Preprocess`：将输入图像变换为模型需要的尺寸和格式
- `Detect`：调用 AXCL、TensorRT 或 MIGraphX 后端生成装甲板候选
- `CornerRefine`：使用传统视觉方法精修灯条和角点
- `Predictor`：执行颜色筛选、PnP、目标选择、跟踪和运动预测
- `Planner`：计算弹道补偿、云台目标和时间序列指令
- `TimedSerial`：按固定周期向串口驱动发送最新指令

流水线使用固定数量的 `ThreadDataPack` 在阶段之间传递数据。串口状态通过消息桥提供给传感器和预测模块，规划结果通过消息桥交给 `TimedSerial`。

## 代码导航

| 主题 | 入口 |
| --- | --- |
| 程序初始化和线程编排 | `main.cpp`、`main.hpp` |
| 流水线和任务生命周期 | `common/pipeline.hpp` |
| 数据结构和枚举 | `common/datatype.hpp` |
| 配置解析 | `common/cmd_parser/` |
| 相机和视频输入 | `sensor/` |
| 图像预处理和检测 | `detect/` |
| 坐标变换和滤波器 | `mathutils/` |
| 目标选择和跟踪 | `predict/` |
| 弹道和云台规划 | `planner/` |
| 串口接口和驱动 | `timedserial/` |

## AX650 部署

AX650 用户使用[AX650 环境配置仓库](https://github.com/Astra-Whale/SHtech_auto_aim_AX650-EnvCfg)准备系统环境。该仓库面向上科大定制 AX650 镜像，负责安装 AXCL SDK、MVS、RMCVSerial 和通用编译依赖，并提供 Auto-Aim 构建入口。

```bash
git clone --branch for_2026_open_source \
    https://github.com/Astra-Whale/SHtech_auto_aim_AX650-EnvCfg.git
cd SHtech_auto_aim_AX650-EnvCfg
bash AutoInstall.sh
```

安装脚本包含交互式步骤。主线编译默认包含海康支持，因此 AX650 部署应安装 MVS，不要跳过 MVS 安装。脚本当前会获取 GitHub 公共版本。内部开发需要在环境准备完成后，切换到目标 GitLab 分支和 commit。

完成环境准备后，在 Auto-Aim 仓库根目录执行构建：

```bash
cmake -S . -B build -DINFERENCE_BACKEND=AXCL
cmake --build build -j4
```

编译阶段固定包含海康支持。运行阶段通过 `launch.cfg` 选择相机或视频输入：

| 模式 | `source` | `port` | 说明 |
| --- | --- | --- | --- |
| AX650 实机 | `0` | 实际串口路径 | 使用海康相机和下位机状态 |
| AX650 离线演示 | `test.avi` | `None` | 使用仓库视频和 `MockDriver` |

两种模式使用同一份包含海康支持的构建产物。离线演示只需要修改运行配置，不需要重新关闭海康编译。

## 构建后端

根工程通过 `INFERENCE_BACKEND` 选择推理后端：

| 取值 | 运行平台 | 主要依赖 |
| --- | --- | --- |
| `AXCL` | AX650 | AXCL SDK，通常位于 `/soc` |
| `TRT` | CUDA 设备 | CUDA 和 TensorRT |
| `ONNX` | ROCm 设备 | MIGraphX 和 ROCm |

通用依赖包括 OpenCV、Eigen3 和 RMCVSerial。启用海康相机时还需要 MVS 运行库，并设置 `MVS_PATH`。

示例配置命令：

```bash
cmake -S . -B build -DINFERENCE_BACKEND=AXCL
cmake --build build -j
```

根据目标平台，将 `AXCL` 替换为 `TRT` 或 `ONNX`。主线构建默认启用海康支持。只有在明确不需要海康运行库的特殊主机环境中，才使用 `-DUSE_HIKCAM=OFF`。当前主机未必具备对应 SDK，因此配置成功不代表目标后端一定能完成编译。

## 配置和输入

程序从仓库根目录的 `launch.cfg` 读取配置。常用字段如下：

| 字段 | 说明 |
| --- | --- |
| `source` | `0` 表示海康相机，其他值表示视频文件路径 |
| `port` | 串口设备路径，`None` 或空值使用 `MockDriver` |
| `model` | 模型文件路径 |
| `camera_para` | 相机内参和相机到 IMU 的外参 |
| `planner_para` | Planner 参数文件 |
| `flip` | 是否旋转输入图像 |

相机配置位于 `asset/camParam/`，Planner 配置位于 `asset/plannerParam/`。这些参数与相机、镜头和机械安装方式相关，不能跨设备直接复用。

### 串口指令和射速

当前 `TimedSerial` 到 `UartDriver` 的实机路径按以下顺序传递控制字段：

```text
yaw, pitch, yaw_speed, pitch_speed, yaw_acc, pitch_acc, distance, shoot, target_id
```

这个顺序与 `advv_detection_t` 的串口字段顺序一致。抽象 `SerialInterface` 和 `MockDriver` 仍使用另一组历史参数名，当前不改变 `UartDriver` 实机路径的传输结果。新增驱动时应按上述字段语义实现，不要只根据位置参数名推断含义。

Uart 路径会读取 MCU 姿态包中的 `shoot_speed`，随后将 `robot_speed_mps` 设置为固定的 `24.5 m/s`。Planner 当前使用这个固定值进行弹道和延迟计算。MockDriver 的初始射速为 `28.0 m/s`。因此，当前项目使用固定射速策略，不提供实时射速合同。协议字段含义确认后，再决定是否接入实时射速。

## 模型和示范资产

仓库保留当前模型和离线输入：

- `asset/models/SKD250526.axmodel`
- `asset/models/SKD250526.onnx`
- `asset/models/SZU0526_fp32input_512x640_nopre_fixoutput.onnx`
- `test.avi`

模型文件对应不同平台和输出约定：

| 模型 | 对应后端 | 用途 |
| --- | --- | --- |
| `SKD250526.axmodel` | `AXCL` | AX650 主路径 |
| `SKD250526.onnx` | `TRT` | TensorRT 路径 |
| `SZU0526_fp32input_512x640_nopre_fixoutput.onnx` | `ONNX` | MIGraphX 路径 |

更换 `INFERENCE_BACKEND` 时，需要同时确认输入尺寸、颜色格式、输出张量、类别表和后处理方式，不能只替换一个 CMake 参数。`ONNX` 后端使用 `SZU0526` 模型，不使用 `SKD250526.onnx` 的输出约定。

### 类别编号和装甲尺寸

检测结果中的 `color_id` 约定为 `0=红色`、`1=蓝色`、`2=灰色`。公共 `tag_id` 约定为 `0=无目标`、`1–7=机器人`、`8=前哨站`、`9=基地`。

AXCL 和 TensorRT 当前直接保留模型输出的原始类别编号。MIGraphX 的 `SZU0526` 路径会把原始类别映射到上述公共编号：原始 `0` 映射为 `7`，原始 `6` 映射为 `8`，原始 `7` 和 `8` 映射为 `9`，其他机器人类别保持原值。

PnP 当前根据 `armor_number` 选择大、小装甲板模型。实现把 `0`、`1`、`8` 判定为大装甲板，其余编号判定为小装甲板。训练标签和比赛目标编号的最终对应关系仍需结合模型标签表确认，本段描述的是当前代码行为，不是新的标签规范。

`test.avi` 用于离线演示和回放。视频和模型的来源、授权范围以及最终公开版本的哈希信息仍需在发布前补齐。

## 运行

完成构建后，在仓库根目录运行：

```bash
./build/auto-aim
```

无硬件模式需要在 `launch.cfg` 中将 `source` 设置为 `test.avi`，将 `port` 设置为 `None`，并选择与构建后端匹配的模型。该模式仍使用包含海康支持的构建产物。

## 系统服务与开机自启动

`install_service.py` 生成 systemd 服务和启动别名。安装服务不会自动启用开机自启动，需要单独执行 `autoaim-enable`。该脚本使用固定目录和较高权限，适合内部实验，不属于推荐的公开部署路径。

```bash
sudo python3 install_service.py
source ~/.bashrc

# 设置开机自启动
autoaim-enable

# 立即启动
autoaim-start
```

常用服务命令如下：

| 命令 | 作用 |
| --- | --- |
| `autoaim-enable` | 设置开机自启动 |
| `autoaim-disable` | 取消开机自启动 |
| `autoaim-start` | 立即启动服务 |
| `autoaim-stop` | 停止服务 |
| `autoaim-status` | 查看服务状态 |

前台运行和视频演示通过直接执行 `./build/auto-aim` 完成。服务部署前需要确认安装目录、日志权限和停止行为符合目标设备要求。

## 当前边界

- 项目主线面向 AX650 和 RoboMaster 自瞄视觉链路
- MCU 固件、云台控制器和发射机构不在本仓库中
- 相机标定、串口协议和机械安装需要结合实际设备确认
- 不同推理后端不是完全可互换的实现
- 当前版本不包含可视化扩展及其第三方 SDK
- 性能和命中率取决于模型、相机、机械、参数和测试场景，本 README 不给出无条件指标
- 代码许可证、第三方声明和模型授权说明尚未统一，公开发布前必须补齐

## 相关依赖

- [RMCVSerial](https://gitlab.rmshtech.com/computer-vision/tools/rmcv_serial)：内部串口库
- 海康 MVS：相机运行库
- [AX650 环境配置](https://github.com/Astra-Whale/SHtech_auto_aim_AX650-EnvCfg)：AX650 平台环境和部署材料
- [算法扫盲文档](https://fcn47qghdcqf.feishu.cn/wiki/Hcw1wxTMZicx0xkinuQcKHetn5d?from=from_copylink)：飞书预印本

内部版本优先保证代码、配置和 README 的对应关系。公开版本将在此基础上移除内部依赖说明，并补充完整的许可证、来源和发布版本信息。
