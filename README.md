# Auto-aim

TODO: 实装预测功能 实装串口通信 替换Log实现 替换cmd_praser实现



项目依赖: 

* CUDA10.2
* TensorRT8
* gcc/g++-9: ubuntu18.04 设备均需手动升级
* Ceres-Solver: 计算库，安装方式: [编译安装](https://github.com/ceres-solver/ceres-solver) (Not necessary so far)
* OpenCV4: 计算机视觉库，安装方式: [编译安装](https://github.com/opencv/opencv.git)
* OpenCV_contrib: OpenCV 额外模块，安装方式: 和 OpenCV 一起[编译安装](https://github.com/opencv/opencv_contrib.git)
* CatKin: ROS组件，串口通信依赖，安装方式: apt install catkin
* libserial: 串口通信库，安装方式: [编译安装](https://gitlab.rmshtech.tk/hegq/libserial)
* MVS: 海康摄像头驱动，安装方式: [下载安装](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)
* [配置脚本参考](http://gitlab.rmshtech.tk/computer-vision/nvidianx-environment-config)
---

更新记录: 

* 2021-10-02: 代码框架初步搭建完成
* 2021-11-20: 封装坐标转换相关代码，实现静态目标预测全部功能
* 2021-12-10: 调整坐标转换封装结构，实现动态目标二阶线性预测

<br>

排坑指南: 

-   MVS安装完成后需在`~/.bashrc`中设置`MVS_PATH`指向`libMvControl.so`所在文件夹。
-   gcc/g++-9: 本项目中如果 gcc/g++ 版本不对，那么会出现以下几个错误
    -   `autoaim/detector/TRTModule.hpp:83` 中 filesystem 报错
    -   `autoaim/detector/TRTModule.hpp:17-18` 中 default 定义错误



# 简介

功能: 

-   视频流输入: 视频文件输入(√) 海康摄像头输入(√)
-   装甲板四交点检测(√) 基于交大开源方案
-   预测: TBD
-   串口通讯: 基于 libserial 和大疆裁判系统通信协议
-   从 launch.cfg 读取运行配置

硬件: 

-   NX: 自带 CUDA + TRT 。
-   camera: 海康工业摄像头
-   TTL: 板载



# 代码框架

```txt
.
├── CMakeLists.txt: 开启了ccahe加速编译，只重新编译修改部分，默认开启编译优化选项
├── README.md: 本文档
├── asset
│   ├── autoaim-param.yml: 自瞄参数文件，前一半是预测参数，一般无需修改，后一半是反陀螺参数，还没有仔细试过
│   ├── camera-param.yml: 相机标定文件，通过 calibrate.py 任务生成
│   ├── model-opt-4.cache: 使用的模型文件的cache缓存，如果第一次使用，会自动生成，在 nx 上的时间约为半小时
│   └── model-opt-4.onnx: 模型文件: 本赛季使用
├── comm: 串口通信相关
│   ├── CMakeLists.txt
│   ├── comm.hpp\cpp: 串口通信类
│   ├── crc.hpp: crc校验
│   └── protocol.hpp\cpp: 大疆裁判系统通信协议封解包代码
├── common: 通用工具
│   ├── CMakeLists.txt
│   ├── common.hpp: 总头文件
│   ├── pipline.hpp: 线程间通信 任务基类
│   ├── robot_status.hpp: 机器人状态 通用数据交换结构
│   ├── cmd_parser: cfg文件读取相关
│   │   ├── cmd_parser.cpp
│   │   └── cmd_parser.hpp
│   ├── log: log记录相关
│   │   ├── log.cpp
│   │   └── log.hpp: LOG{M,W,E}_{S,F}向屏幕/文件输出消息/警告/错误
│   └── timer: 计时 测fps
│       ├── timer.hpp
│       └── timer.cpp
├── detect: 装甲板四角点检测
│   ├── CMakeLists.txt
│   ├── detect.hpp\cpp: 装甲板四角点检测任务类 派生自任务基类 init时实例化模型
│   └── TRTModule.hpp\cpp: 基于TensorRT C++ API构建推理环境和进行推理
├── predict: 预测
│   ├── CMakeLists.txt
│   ├── predict.hpp\cpp: 预测任务类 派生自任务基类 TBD
│   ├── tools.hpp: 坐标转换和弹道补偿相关函数
|   |── StaticPredictor.hpp\cpp: 静态目标预测
|   └── LinearPredictor.hpp\cpp: 动态目标线性预测
├── sensor: 传感器相关
│   ├── CMakeLists.txt
│   ├── sensor.hpp\cpp: 传感器数据获取任务类 派生自任务基类 init时打开传感器接口
│   ├── cam_wrapper.hpp: 视频流基类
│   ├── i2c: I2C通信支持库
│   │   └── i2c.h\c
│   ├── UartIMU: 串口位姿数据
│   │   └── uartimu.hpp\cpp
│   ├── BMI160: 板载IMU位姿数据
│   │   └── BMI160.hpp\cpp
│   ├── video: 视频图像读取
│   │   └── video_wrapper.hpp\cpp
│   └── hikcam: 相机图像读取
│       ├── hikcam_wrapper.hpp\cpp
│       └── ...: MVS相关头文件
├── build: 编译目录
├── data: 数据存储部分
├── main.cpp: 主函数入口
└── main.hpp: 
```



# 项目介绍

## 线程间通信

通过`pipline_queue_t`实现的报文缓存队列控制自瞄流水线，报文对象以`shared_ptr`智能指针的形式在线程间循环使用。报文对象中为线程间通信所需的所有变量预留了空间，节省了时间开销。

## 任务基类

通过`BasicTask`实现了基于`pipline`的线程生命周期管理。定义了`init`,`setdebug`,`setshow`,`stop`等方法并重载了`()`运算符，允许函数式调起任务。

## 数据IO线程

通过`sensor::Sensor`类管理并对外提供数据IO功能。通过 ***lauch.cfg*** 可管理数据源。

## 装甲板检测线程

通过`detect::Detect`类管理 **TensorRT** 推理资源。使用 ***lauch.cfg*** 中指定的onnx文件创建并缓存推理网络，基于 **OpenCV** 和 **TensorRT** 进行图像预处理、图像推理、数据后处理。

## 目标预测线程

通过`predict::Predict`类管理目标预测过程。在命名空间中实现了提供静态目标预测的`StaticPredictor`子类和提供二阶线性预测的`LinearPredictor`子类，利用 **Eigen** 和 **OpenCV** 的高效封装实现装甲板筛选、PNP解算、坐标变换、位置预测、弹道补偿等核心功能。
