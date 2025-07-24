# OpenLoong 动态控制

## 基于 MPC 和 WBC 的类人机器人运动控制框架

请访问 🐉 OpenLoong 开源代码库！

OpenLoong 开源项目是由上海类人机器人有限公司、上海类人机器人制造创新中心和 OpenAtom 基金会共同运营的合作倡议。该代码库提供了基于 MPC（模型预测控制）和 WBC（全身控制）的类人机器人控制框架，可在 Mujoco 仿真平台上部署。基于上海类人机器人创新中心的“青龙”机器人模型，提供了三个运动示例：[行走](https://github.com/loongOpen/Openloong-dyn-control/blob/main/demo/walk_wbc.cpp)、[跳跃](https://github.com/loongOpen/Openloong-dyn-control/blob/main/demo/jump_mpc.cpp) 和 [盲障碍踩踏](https://github.com/loongOpen/Openloong-dyn-control/blob/main/demo/walk_mpc_wbc.cpp)。物理原型已实现 <b>行走</b> 和 <b>盲障碍踩踏</b> 动作。

📖 **[阅读中文版](./README-zh.md)**

## 项目特点

- **易于部署** 提供了一个全面的解决方案，用于部署代码执行环境，使用户能够轻松配置所需的工作环境。该代码库包括主要依赖项，消除了大量第三方库安装的需求，简化了整个部署过程。
- **可扩展** 控制框架结构采用分层模块化设计，旨在增强系统的可维护性和可扩展性。系统的每个功能模块具有明确的逻辑和功能边界，为二次开发提供了更友好的开发者环境，使开发者更容易自定义和扩展系统功能。
- **易于理解** 代码结构简洁，遵循基于功能的模块封装原则。使用总线进行模块间的数据交互，减少封装冗余，帮助降低代码复杂性。算法实现遵循简单的“读-计算-写”逻辑，提高了代码的可读性。

<center><img src="./assets/行走.gif" alt="行走" style="zoom:50%;" /><img src="./assets/踩障碍.gif" alt="踩障碍" style="zoom:50%;" /></center>

## 更新日志

2024.06.29

1. 添加了两个新演示，`walk_wbc_joystick` 和 `walk_mpc_wbc_joystick`，允许通过键盘控制机器人运动并启用转向功能。

2024.08.12

1. 修复了 MuJoCo 中传感器数据提取 ID 不正确的问题。感谢 Xunlong Software 报告此问题。
2. 修正了 MPC 中 c 矩阵定义的维度错误。感谢 @geekloong 和 @yichuanku 报告此问题。
3. 修复了 WBC 优先级计算第一优先级的计算错误。感谢 @1190201119 报告此问题。
4. 修改了 MPC 中的成本函数。

2024.09.11

1. 添加了一个名为 "low\_damping\_model" 的新分支，与物理原型的关节响应紧密对齐。该分支提供了两个演示：`walk_wbc_joystick` 和 `walk_mpc_wbc_joystick`。
2. 添加了 **模型替换** 文档[Tutorial](https://github.com/loongOpen/Openloong-dyn-control/blob/main/Tutorial.md)。

## 环境安装

**环境建议**

- 操作系统：Ubuntu 22.04.4 LTS
- 编译器：g++ 11.4.0

**依赖安装**

该代码库基于 MuJoCo 进行“青龙”类人机器人仿真测试。代码库还包括 MuJoCo 仿真引擎、Pinocchio 动力学库、Eigen、Quill 日志工具、GLFW 图形库和 JsonCpp 解析库。然而，仿真接口需要系统支持 OpenGL，需进行安装。

```bash
# 更新并安装依赖
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

## 安装指南

**代码获取与编译**

```bash
# 克隆
git clone https://github.com/loongOpen/Openloong-dyn-control.git

# 编译
cd Openloong-dyn-control
mkdir build
cd build
cmake ..
make

# Mujoco 仿真
./walk_mpc_wbc # 或 ./walk_wbc 或 ./jump_mpc
```

**仿真性能**

<img src="./assets/demo.png" alt="demo" style="zoom:50%;" />

## **代码说明**

请参考该代码的 API 接口。[文档](https://www.openloong.org.cn/pages/api/html/index.html)和[Wiki](https://www.openloong.org.cn/pages/wiki/html/index.html)。

**主要前缀和后缀说明**

| 前缀和后缀         | 说明                      |
| ---------------- | -------------------------- |
| *_L, _W*         | 在局部框架、在全局框架    |
| *fe_*            | 脚的末端                  |
| *_L, _l, _R, _r* | 左|右                       |
| *swing,* *sw*    | 摆动腿                    |
| *stance,* *st*   | 支撑腿                    |
| *eul, rpy*       | 姿态角                    |
| *omega*          | 角速度                    |
| *pos*            | 位置                      |
| *vel*            | 线速度                    |
| *tor**, tau*     | 力矩                      |
| *base*           | *BaseLink*                |
| *_des*           | 期望值                    |
| *_cur*           | 当前值                    |
| *_rot*           | 坐标变换矩阵              |

## 开发指南

**关键控制参数**

- MPC 权重

```cpp
// MPC.h
void set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
//*u_weight* ：系统输入的最小权重
//*L_diag* ：系统状态和期望误差的权重，顺序为 eul, pos, omega, vel
//*K_diag* ：系统输入的权重，顺序为 fl, tl, fr, tr
```

- WBC 任务优先级

```cpp
// WBC_QP.cpp
std::vector<std::string> taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
// 调整任务优先级顺序
```

- WBC 权重

```cpp
// PriorityTasks.h
Eigen::MatrixXd Kp;                // 特定 WBC 优先级的位置信息误差权重
Eigen::MatrixXd Kd;                // 特定 WBC 优先级的速度信息误差权重
// WBC_QP.h
Eigen::MatrixXd Q1;                // 外部接触力和期望误差权重，顺序为 fl, tl, fr, tr
Eigen::MatrixXd Q2;                // 关节加速度和期望误差权重
```

- 摆动腿轨迹

```cpp
// FootPlacement.h
double kp_vx;                                 // x 方向脚的位置调整参数
double kp_vy;                                 // y 方向脚的位置调整参数
double kp_wz;                                 // z 方向脚的位置调整参数
double stepHeight;                            // 步高
// FootPlacement.cpp
double FootPlacement::Trajectory(double phase, double des1, double des2);        // 摆动腿的 z 方向轨迹
// phase：摆动阶段达到最高点
// des1：最高轨迹位置
// des2：最终轨迹位置
```

- 步态控制

```cpp
// GaitScheduler.h
double tSwing;                                         // 单步持续时间  
double FzThrehold;                                     // 地面接触力阈值
// GaitScheduler.cpp
DataBus::LegState legState=DataBus::RS;                // 初始摆动腿
```

- 关节参数

```json
// JointCtrConfig.json
   "Joint-ankle-l-pitch" : {
      "PVT_LPF_Fc" : 20,
      "kd" : 5.0,
      "kp" : 50.0,
      "maxPos" : 0.61087,
      "maxSpeed" : 48.8,
      "maxTorque" : 58.5,
      "minPos" : -0.43644
   }
```

**模型替换指南**

请参考模型替换的[Tutorial](https://github.com/loongOpen/Openloong-dyn-control/blob/main/Tutorial.md)文档。

## 参考文献

[1] D. Kim, J. D. Carlo, B. Katz, G. Bledt, S. Kim, Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control. arXiv:1909.06586 (2019).

[2] Kim D, Jorgensen S J, Lee J, et al. Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control. arXiv:1901.08100 (2020).

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the MIT cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9.

[4] 卞泽坤, 王兴兴. 四足机器人控制算法: 建模、控制与实践[M]. 机械工业出版社, 2023

## 引用格式

如果您使用了该开源项目中的代码，请按以下格式引用：

```javascript
@software{Robot2024OpenLoong,
  author = {Humanoid Robot (Shanghai) Co., Ltd},
  title = {{OpenLoong-DynamicsControl: 基于 MPC 和 WBC 的类人机器人运动控制框架}},
  url = {https://github.com/loongOpen/Openloong-dyn-control.git},
  year = {2024}
}
```

## 联系信息

欢迎开发者为该代码库的优化和改进做出贡献！

[💬 开始讨论](https://github.com/orgs/loongOpen/discussions) | [📝 报告问题](https://github.com/loongOpen/Openloong-dyn-control/issues) | [📨 提交变更请求](https://github.com/loongOpen/Openloong-dyn-control/pulls)

如有任何关于本代码的问题或建议，请联系 <open@openloong.org.cn>
