# 依赖与示范资产

这份文档说明 `auto-aim` 当前实现实际使用的系统依赖、外部库、内置组件和示范资产。它补充根目录 `README.md`，不替代 EnvCfg 安装脚本，也不把未充分维护的后端描述为主线能力。

## 支持范围

| 类别 | 当前定位 | 说明 |
| --- | --- | --- |
| AXCL | 主线 | 面向 AX650，使用 `asset/models/SKD250526.axmodel` |
| TensorRT | 保留后端 | 需要 CUDA 和 TensorRT，维护与验证不充分 |
| MIGraphX | 保留后端 | CMake 取值为 `ONNX`，使用 `SZU0526` 模型，维护与验证不充分 |
| 视频离线模式 | 示范模式 | 使用 `test.avi` 和 `MockDriver`，不需要连接相机或下位机 |
| `tools/` | 辅助工具 | 用于标定、日志分析和算法实验，不属于主流程支持面 |

## 依赖总览

| 依赖 | 代码位置或来源 | 使用模块 | 准备方式 | 当前边界 |
| --- | --- | --- | --- | --- |
| OpenCV | 系统包 | 全局图像和矩阵处理 | `find_package(OpenCV)` | 版本未锁定 |
| Eigen3 | 系统包 | 坐标变换、滤波和规划 | `find_package(Eigen3)` | 版本未锁定 |
| Boost | RMCVSerial 的构建依赖 | `timedserial` 间接使用 | EnvCfg 构建 RMCVSerial 时安装 | 主工程没有独立的 Boost 检查 |
| AXCL | AX650 基础镜像和 `/soc` | `detect/AXCL` | EnvCfg 解包 AXCL 头文件 | 运行库不在 `auto-aim` 仓库中 |
| Hikvision MVS | `sensor/hikcam` 头文件和 EnvCfg 安装包 | `sensor` | EnvCfg 安装 MVS 并设置 `MVS_PATH` | 版本和授权范围尚未记录 |
| RMCVSerial | 内部 GitLab `V1.1.1` | `timedserial/UartIMU` | EnvCfg 下载、构建并安装到 `/usr/local` | 压缩包版本与 CMake 内部版本不一致 |
| TinyMPC | `planner/tinympc` | `planner` | 随仓库构建 `tinympcstatic` | 当前是改动后的源码快照，未记录源 commit |
| CUDA、TensorRT | 系统运行时 | `detect/TensorRT` | 目标设备自行准备 | 只在 `INFERENCE_BACKEND=TRT` 时使用 |
| ROCm、MIGraphX | `/opt/rocm` | `detect/ONNX` | 目标设备自行准备 | 只在 `INFERENCE_BACKEND=ONNX` 时使用 |

主线构建默认启用海康支持。编译阶段会包含 `sensor/hikcam`，运行阶段由 `launch.cfg` 的 `source` 选择海康相机或 `test.avi`。离线模式不代表可以省略 AX650 目标环境中的 MVS 安装。

## EnvCfg 的职责

[AX650 环境配置仓库](https://github.com/Astra-Whale/SHtech_auto_aim_AX650-EnvCfg)面向上科大定制 AX650 镜像。当前 `for_2026_open_source` 分支的 `AutoInstall.sh` 执行以下工作：

- 安装 GCC、CMake、OpenCV、Eigen3 和 Boost 等通用构建依赖
- 将 `sources/axcl_inc.tar` 解包到 `/soc`，补充 AXCL 头文件
- 交互式安装 `sources/MVS_aarch.deb`，并设置 `MVS_PATH`
- 从内部 GitLab 下载 RMCVSerial `V1.1.1`，构建后安装到 `/usr/local`
- 获取 GitHub 公共版 `auto-aim` 并执行 AXCL 构建

AXCL 运行库、AX650 固件和设备侧驱动由基础镜像或目标设备提供。EnvCfg 不会把这些运行时内容转换为通用 Ubuntu 环境。

脚本当前没有为下载内容提供统一的 commit、版本清单或校验和。脚本历史还包含部署凭据。内部使用时应把凭据轮换和历史清理列为独立任务，文档不展示凭据内容。

环境准备完成后，重新加载环境变量，再在 `auto-aim` 根目录构建：

```bash
source ~/.bashrc
cmake -S . -B build -DINFERENCE_BACKEND=AXCL
cmake --build build -j4
```

## 外部库

### RMCVSerial

`timedserial/UartIMU/uart_driver.hpp`直接使用 `drivers::RMCVSerial`。EnvCfg 当前从内部 GitLab 的 `V1.1.1` 标签下载源码，并通过 CMake 安装共享库和头文件。

当前审计结果：

- 源码使用 Boost.Asio，C++ 标准设置为 C++14
- 压缩包文件没有独立 README、LICENSE 或 NOTICE
- 头文件和部分源文件带有 Apache 2.0 许可证说明，并列出 LeoDrive、Autoware Foundation 和 Trimble 的版权信息
- CMake 项目内部版本仍写为 `1.0.0`，与下载标签 `V1.1.1` 不一致
- `timedserial` 通过链接器名称 `RMCVSerial` 查找安装结果，没有使用导出的 CMake package 配置

内部文档可以把 `V1.1.1`作为当前依赖版本。公开发布前仍需补齐源 commit、许可证文件和第三方声明。

### Hikvision MVS

`sensor/hikcam/`保存编译所需的 MVS 头文件快照。`hikcam_wrapper.cpp`调用 `MV_CC_*`接口完成 USB 相机枚举、采集、像素格式处理和资源释放。动态库由 EnvCfg 安装到目标设备，CMake 通过 `MVS_PATH` 查找。

当前仓库没有记录 MVS SDK 的精确版本。MVS 安装包也不属于 `auto-aim` 主仓库。公开版本应根据海康分发条款单独确认头文件和二进制文件的授权范围。

### TinyMPC

`planner/tinympc/`直接编译为 `tinympcstatic`，`planner/Planner.cpp`使用它建立偏航和俯仰两个 MPC 求解器。当前 Planner 的预测时域为 `HORIZON = 100`，每个求解器限制为 10 次迭代。

这份源码可以确认来自 [TinyMPC 官方仓库](https://github.com/TinyMPC/TinyMPC)，但本仓库没有记录上游 commit。当前快照与上游代码存在差异，至少删减了时间变化线性约束相关实现。后续应记录快照来源、修改范围和许可证归属，并在公开版本加入对应 NOTICE。

## 模型与离线资产

代码按后端约定使用以下文件：

| 文件 | 后端 | 代码侧输入和输出约定 | 用途 |
| --- | --- | --- | --- |
| `asset/models/SKD250526.axmodel` | AXCL | 640 × 512，按 6720 个候选和每候选 21 个值解析 | AX650 主路径 |
| `asset/models/SKD250526.onnx` | TensorRT | 640 × 512，按 6720 个候选和每候选 21 个值解析 | TensorRT 路径 |
| `asset/models/SZU0526_fp32input_512x640_nopre_fixoutput.onnx` | MIGraphX | 640 × 512，代码按 20160 个候选和每候选 22 个值解析 | `ONNX` 后端路径 |
| `test.avi` | 视频输入 | 由 `sensor/video`读取 | 离线回放和示范 |

当前工作树的 SHA-256：

```text
ba89b91f7df6c36f4260fec8080bd6ff8596e3114c017df90e2fab57d3ebb154  asset/models/SKD250526.axmodel
0fdea1e9b894c2f2d24ae6a5e78b57cbe6acf20d62b8636fb66f68be63635426  asset/models/SKD250526.onnx
582dde276f67513e95ab87c5480e025bf102c508b541517609a48c37385d046e  asset/models/SZU0526_fp32input_512x640_nopre_fixoutput.onnx
7faf6d9a1a19e04af482748dfb49f66069043057f29b9cb424d2f5c623508742  test.avi
```

模型来源、训练数据、标签表和授权范围尚未形成完整记录。哈希只能确认文件内容，不代表模型或视频已经获得公开分发授权。

### 标签与装甲尺寸

当前代码侧的公共编号如下：

- `color_id`：`0` 表示红色，`1` 表示蓝色，`2` 表示灰色
- `tag_id`：`0` 表示无目标，`1` 到 `7` 表示机器人，`8` 表示前哨站，`9` 表示基地
- PnP 使用 `armor_number` 判断装甲尺寸。`0`、`1` 和 `8` 归为大装甲板，其余编号归为小装甲板

AXCL 和 TensorRT 保留模型输出的类别编号。MIGraphX 的 `SZU0526`路径会执行类别和颜色映射。训练标签与比赛目标编号的最终对应关系仍需结合模型标签表确认。

## 辅助工具

| 文件 | 输入或额外依赖 | 当前状态 |
| --- | --- | --- |
| `tools/handeye_calibration.py` | `handeye_calibration_data/`、棋盘格图像、`euler_*.txt`、OpenCV 和 NumPy | 示例脚本，内含固定相机参数 |
| `tools/pipeline_visualize.py` | 当前工作目录下的 `25.txt`、Matplotlib | 日志格式依赖外部文件，Stage View 标记为暂停使用 |
| `tools/data_plotter.py` | 已构建的 `./build/auto-aim`、Matplotlib Tk 后端 | 实时观察脚本，不提供独立环境文件 |
| `tools/predictor_simulation.py` | OpenCV、NumPy、Matplotlib 和 Python `tinympc` 包 | Python 包与 C++ vendored TinyMPC 不是同一安装路径 |

这些脚本不参与 `auto-aim` 主程序构建。运行前需要准备脚本所需的输入文件和 Python 依赖。仓库不把现场日志和临时数据作为工具输入的一部分提交。

## 验证状态与待办

本次审计只做静态检查。当前 Ubuntu 环境没有 `/soc`、MVS、RMCVSerial、ROCm 或 TensorRT 运行库，因此没有执行 AX650 构建或正式单元测试。目标设备上的运行验证仍需由对应环境完成。

后续按以下顺序补齐：

- 记录 EnvCfg、RMCVSerial 和 TinyMPC 的精确版本或 commit
- 补齐 MVS、模型和 `test.avi` 的来源与授权信息
- 为 RMCVSerial 和 TinyMPC 形成第三方声明
- 为 EnvCfg 下载内容补充校验和，轮换并清理历史部署凭据
- 为辅助工具补充最小依赖说明，或明确标记为实验脚本
