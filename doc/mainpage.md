# 基于 MPC 和 WBC 的类人运动控制框架

欢迎来到 OpenLoong 动态控制项目的文档！

OpenLoong 动态控制项目是由上海类人机器人有限公司、上海类人机器人制造创新中心和 OpenAtom 基金会主导的开源项目。

该项目基于上海类人机器人创新中心的“AzureLoong”机器人模型，提供了一个基于 MPC-WBC 的控制框架，运行在 Mujoco 上，并展示了包括行走、跳跃和盲目跨越障碍物的三个示例。[项目主页](http://aa.com)

## 属性
**易于部署**：该项目提供了一个全面的类人控制解决方案。代码是自包含的，用户可以轻松配置工作环境，无需安装其他依赖项，从而简化了部署过程。

**易于理解**：类人控制框架结构采用层次化模块设计，提高了系统的可维护性和可扩展性。设计为系统的所有模块在逻辑和功能上划定了明确的边界，为二次开发提供了更友好的环境，使开发者能够更轻松地自定义和扩展功能。

**易于开发**：代码结构简单，遵循功能模块封装的设计原则，使用总线进行模块间的数据交互，减少封装冗余，帮助降低代码复杂性；算法实现采用简单的“读-计算-写”逻辑，提高了代码的可读性。

<img src="figure1.png" alt="im" width="700" height="400" />

# 用户手册
## 环境设置
### 建议平台
- 操作系统：Ubuntu 22.04.4 LTS
- 编译器：GCC 11.4.0
### 安装依赖
此代码库模拟“AzureLoong”类人机器人。代码中已经包含一些依赖项，包括 Mujoco、Pinocchio、Eigen、Quill、GLFW 和 JsonCpp，但仿真仍需要 OpenGL，可以通过执行以下指令安装：
```shell
sudo apt install libglu1-mesa-dev freeglut3-dev
```
## 代码获取与编译
代码可以在 Gitee 下载，或者您可以使用 git 克隆代码库：
```shell
# 克隆
git clone https://atomgit.com/openloong/openloong-dyn-control.git

# 编译
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make

# Mujoco 仿真
./Walk_mpc_wbc # 或 ./Walk_wbc 或 ./Jump_mpc
```

## 代码说明
[这是](http://cc.com)代码的 API 文档。

### 缩写
| 前缀/后缀    | 说明                    |
| ------------ | ------------------------ |
| *L*, *W*     | 局部框架，世界框架      |
| *fe*         | 脚的末端                |
| *L*, *l*, *R*, *r* | 左，右                 |
| *swing*, *sw* | 摆动腿                  |
| *stance*, *st* | 支撑腿                  |
| *eul*, *rpy* | 用欧拉角表示的角位置    |
| *omega*      | 角速度                  |
| *pos*        | 线性位置                |
| *vel*        | 线性速度                |
| *tor*, *tau* | 关节处的力矩            |
| *base*       | 基座链接                |
| *des*        | 期望值                  |
| *cur*        | 当前值                  |
| *rot*        | 旋转矩阵                |

## 开发者指南

### 关键控制参数
- MPC 参数
```C
// MPC.h
void set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
// *u_weight* : 控制输入的最小权重
// *L_diag* : 与期望值相比的误差权重，顺序为 (eul, pos, omega, vel)
// *K_diag* : 控制输入的权重，顺序为 (fl, tl, fr, tr)
```

- WBC 优先级
```C
// WBC_QP.cpp
std::vector<std::string> taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
// 在此添加任务或调整优先级
```

- WBC 权重
```C
// PriorityTasks.h
Eigen::MatrixXd Kp;                // 位置误差权重
Eigen::MatrixXd Kd;                // 速度误差权重
// WBC_QP.h
Eigen::MatrixXd Q1;                // 与期望接触力的误差权重，顺序为 (fl, tl, fr, tr)
Eigen::MatrixXd Q2;                // 加速度跟踪误差的权重
```

- 摆动腿轨迹
```C
// FootPlacement.h
double kp_vx;                                 // x 方向脚的位置参数
double kp_vy;                                 // y 方向脚的位置参数
double kp_wz;                                 // z 方向姿态参数
double stepHeight;                            // 最大步高

// FootPlacement.cpp
double FootPlacement::Trajectory(double phase, double des1, double des2);        // z 方向姿态轨迹
// phase：达到最高点的阶段
// des1：轨迹中的最高位置
// des2：轨迹的最终位置
```

- 步态控制
```C
// GaitScheduler.h
double tSwing;                                         // 单步时间
double FzThrehold;                                     // 接触地面时的最大力

// GaitScheduler.cpp
DataBus::LegState legState = DataBus::RSt;                // 第一步状态
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

### 替换机器人模型的说明
**1. 模型文件**

a. XML 文件准备

准备机器人的 URDF (.urdf) 文件和网格文件 (.stl)，以添加 Mujoco 编译标签：
```XML
<mujoco>
<compiler
        meshdir="meshes/"
        balanceinertia="true"
        discardvisual="false" />
</mujoco>
```

将工作目录更改为 `mujoco-3.x.x/bin`，运行命令：
```shell
./simulate
```

将 URDF 文件拖入仿真界面，模型正确显示后保存 XML 文件。请注意网格文件的路径。

您还可以参考 Mujoco [文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)来设置标签，如 `compiler`、`option` 或 `asset`，以自定义 `body`、`actuator` 和 `sensor` 等。

| 父标签        | 子标签                    |
| -------------- | ------------------------- |
| *worldbody*    | 定义光源、相机、地面和机器人（包括惯性、关节、自由关节、几何体、位置、相机、光源等） |
| *actuator*     | 定义执行器（电机、位置、速度等） |
| *sensor*       | 定义传感器并调整传感器参数，如噪声 |

b. 替换模型

以“AzureDragon 机器人”为例：在 *base_link* 下，有四个并联连接：头部 *Link_head_*、腰部 *Link_waist_*、左臂 *Link_arm_l_* 和右臂 *Link_arm_r_*。左臂和右臂各有 7 个自由度，头部有 2 个自由度。腰部有 3 个自由度，包括俯仰 *Link_waist_pitch*、滚转 *Link_waist_roll*、偏航 *Link_waist_yaw* 等，左腿和右腿并联连接，每条腿依次连接三个髋关节 *Link_hip* 和一个膝关节 *Link_knee_*，两个踝关节 *Link_ankle_*，总共 6 个自由度。这完成了所有 31 个自由度的配置。

您可以参考此配置并尝试自定义您的配置：
```XML
<worldbody>
    <body name="base_link" pos="x x x">
        <freejoint name="float_base" />
        <body name="body1" pos="x x x">
            <inertial pos="x x x" quat="x x x" mass="x" diaginertia="x x x"/>
            <joint name="joint1" pos="0 0 0" axis="1 0 0" limited="true" range="x x"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="x x x 1" mesh="body1"/>
            <geom type="mesh" rgba="x x x 1" mesh="body1"/>
            <body name="body2" pos="x x x">
                ...
                <joint name="joint2" .../>
                ...
            </body>
        </body>
        <body name="body3" pos="x x x">
            ...
            <joint name="joint3" .../>
            ...
        </body>
    </body>
</worldbody>
```

在这种情况下，两条分支在 *base_link* 下并联连接，一条分支由 *body1* 和 *body2* 串联组成，另一条分支由 *body3* 组成。如果机器人有浮动基座，则在上述名为 *base_link* 的 *body* 下添加自由关节 *freejoint*。如果机器人是固定基座，则移除 *freejoint*。在模型配置阶段，如果需要，可以选择性地隐藏 *freejoint*。

该项目为所有 31 个关节设置了执行器。
```XML
<actuator>
    <motor name="motor1" joint="joint1" gear="x" ctrllimited="true" ctrlrange="x x"/>
    ...
</actuator>
```

用户可以根据机器人的自由度定义相应的执行器。

该项目配置了传感器，如四元数 *framequat*、速度计 *velocimeter*、角速度计 *gyro*、加速度计 *accelerometer*，这些传感器已安装在 *body* 标签中定义的 *site* 上，并可以根据需要添加触觉、力、扭矩、关节位置、关节速度、执行器力等传感器。
```XML
<sensor>
    <framequat name="xx" objtype="site" objname="imu" />
    <velocimeter name="xx" site="imu" />
    <gyro name="xx" site="imu" />
    <accelerometer name="xx" site="imu" />
</sensor>
```
除了自由度配置、执行器配置、传感器配置外，其他更具体的参数修改可以参考 Mujoco 官方 [文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。

**2. 控制代码和 Mujoco 接口**

使用函数 `mj_loadXML`、`mj_makeData` 获取 `mjModel`、`mjData` 结构。您可以参考 [文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html) 获取有关 `mjModel`、`mjData`、`mjOption` 的更多详细信息。
```c
mjModel* mj_model = mj_loadXML("../Models/xxx.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);
```

`mj_model->nv` 是广义速度坐标的维度，即浮动基座的线性速度、角速度以及 31 个旋转型关节的速度。项目框架中与自由度相关的变量对应于 `mj_model->nv-6`，动力学库将根据 URDF 自动获取机器人的自由度维度，所有维度信息都在其中定义。因此，用户无需在程序中手动修改。

由于访问该项目的 *body*、*joint*、*motor* 和其他组件的地址依赖于查询名称字符串并锁定地址，因此当某个组件被修改时，不会影响其他 *body*、*joint* 的数据读取和写入，相比直接索引数字提供了便利。在修改模型中某个自由度的控制参数时，只需修改 *MJ_Interface.h* 中的 `JointName`、*Pin_KinDyn.h* 中的 `motorName`、*PVT_Ctr.h* 中的 `motorName`，以及 *JointCtrConfig.json* 文件中与某个自由度名称对应的变量。例如，要修改 `J_waist_pitch` 的刚度，您需要修改 `J_waist_pitch` 及其在 *JointCtrConfig.json* 中的对应 PD 参数，`J_waist_pitch` 的名称对应于 XML 文件中的 *joint name* 和 *motor name*。

传感器数据地址也是通过查询名称字符串找到地址来访问的，添加或删除传感器可以通过修改 *MJ_Interface.h* 中对应的传感器名称来完成。

# 参考文献
[1] D. Kim, J. D. Carlo, B. Katz, G. Bledt, S. Kim, Highly dynamic quadruped locomotion via whole-body impulse control and model predictive control. arXiv:1909.06586 (2019)。

[2] Kim D, Jorgensen S J, Lee J, et al. Dynamic locomotion for passive-ankle biped robots and humanoids using whole-body locomotion control. arXiv:1901.08100 (2020)。

[3] Di Carlo J, Wensing P M, Katz B, et al. Dynamic locomotion in the MIT Cheetah 3 through convex model-predictive control[C]//2018 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2018: 1-9。

# 引用
```Java
@software{Robot2024OpenLoong,
  author = {Humanoid Robot (Shanghai) Co., Ltd},
  title = {{OpenLoong-DynamicsControl: 基于 MPC 和 WBC 的类人机器人运动控制框架}},
  url = {https://atomgit.com/openloong/openloong-dyn-control.git},
  year = {2024}
}
```
# 联系我们
欢迎开发者参与优化和改进此代码库！

您可以对现有内容进行评论，反馈问题，贡献您的原创内容等。如果您有任何问题或建议，请联系 web@openloong.org.cn
