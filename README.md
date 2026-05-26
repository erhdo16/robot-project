# robot-project

## 项目简介
本项目基于 MATLAB 实现：
- 7自由度机械臂运动学建模
- 5自由度欠驱动手手势控制
- 轨迹规划
- 人机交互（石头剪刀布）/对一些基本手势做出回应

## 项目成员
- 闫嘉怡
- 刘鄢淇
- 胡水沅
- P. Jinjuta

## 项目结构
- arm：机械臂建模
- hand：欠驱动手控制
- trajectory：轨迹规划
- vision：视觉识别
- demo：最终演示

## 开发环境
- MATLAB（2025b及以上）
## 核心仿真与控制
- **Simulink**
- **Robotics System Toolbox (机器人系统工具箱)**
- Simscape & Simscape Multibody（这两个通常一起装）

负责把 URDF 文件变成一个有重力、有碰撞、能动的 3D 物理机器人。
- Control System Toolbox (控制系统工具箱)
## 视觉识别与交互逻辑
- Computer Vision Toolbox (计算机视觉工具箱)
- MATLAB Support Package for USB Webcams
调用摄像头采集画面，提取人手的特征。
- Image Processing Toolbox (图像处理工具箱)

视觉辅助。比如把摄像头的背景去掉，只锁定手部区域，减少干扰。
- Deep Learning Toolbox (深度学习工具箱)
## 时间
第15周 6.2/6.4

## 文件解释
<img width="1166" height="500" alt="image" src="https://github.com/user-attachments/assets/c3ce7da0-d0c1-4abf-b392-692665c869d8" />

## 具体内容
## 1. 右臂 (Right Arm) —— 核心控制对象
### 自由度：7-DOF
任务目标中的主手臂,是一个典型的冗余配置（7个关节控制空间6个维度的位姿）。

| 序号 | 关节名称 | 类型 | 功能描述 |
|:----:|:---------:|:----:|:---------|
| 37 | right_shoulder_pitch | Revolute | 肩部俯仰 前((+)后(-)抬臂 |
| 38 | right_shoulder_roll | Revolute | 肩部外展(+)/内收(-) |
| 39 | right_shoulder_yaw | Revolute | 肩部旋转（大臂旋内+/旋外-） |
| 40 | right_elbow_pitch | Revolute | 肘部俯仰 (屈肘+/伸展-) |
| 41 | right_elbow_yaw | Revolute | 肘部旋转（以小臂为轴(掌心)+/旋后-） |
| 42 | right_wrist_pitch | Revolute | 腕部俯仰(勾手/向掌心-/翘手+)  |
| 43 | right_wrist_yaw | Revolute | 腕部偏航（末端旋转） (向拇指方向偏+|
 
 Yaw-Pitch-Roll

## 2. 右手 (Right Hand) —— 欠驱动手势设计
**结构：11个物理关节，由5个驱动源（电机）耦合控制。需要通过控制电机来带动以下物理关节：**
- 食指 (Index): &nbsp; 44 (2_pitch), &nbsp; 45 (3_pitch) —— 耦合运动
- 小指 (Little): &nbsp; 46 (2_pitch), &nbsp; 47 (3_pitch) —— 耦合运动
- 中指 (Middle): &nbsp; 48 (2_pitch), &nbsp; 49 (3_pitch) —— 耦合运动
- 无名指 (Ring): &nbsp; 50 (2_pitch), &nbsp; 51 (3_pitch) —— 耦合运动
- 大拇指 (Thumb): &nbsp; 52 (1_roll), &nbsp; 53 (2_pitch), &nbsp; 54 (3_pitch) —— 较为复杂的运动

**2_pitch  = 近节（PIP关节）- 主动控制关节 = 靠近手掌的指节**

**3_pitch  = 远节（DIP关节）- 耦合被动关节 = 指尖的指节**

**远节角度 = COUPLE × 近节角度 (COUPLE = 0.7;)**

### 3. 左臂与左手 (Left Arm & Hand) —— 镜像部位
**若需要双臂舞蹈，参数与右侧镜像对应：**
- 左臂 (7-DOF): 关节 19 (left_shoulder_pitch) 到 25 (left_wrist_yaw)。
- 左手: 关节 26 到 36，配置与右手一致。
## 4. 计算机视觉相关部位 (Vision/Head)
在机器人感知和交互中，视觉系统的安装位置决定了相机的“视场角”和“坐标系转换”。

| 序号 | 部位名称 | 关节名称 | 功能描述 |
|:----:|:---------|:---------|:---------|
| 17 | 颈部 | `head_yaw` | 🔄 头部左右转动（水平扫描） |
| 18 | 颈部 | `head_pitch` | ⬆️⬇️ 头部上下俯仰（寻找地面目标或人脸） |

### 补充
共54个关节，但其中2个是fixed关节:
- body7  (llegsite)  —— 左脚固定连接，不进入配置向量
- body14 (rlegsite)  —— 右脚固定连接，不进入配置向量
- 因此配置向量长度 = 54 - 2 = 52
### 总索引
- body 1-6   → 配置 1-6    （左腿6关节）
- body 7     → 跳过（fixed）
- body 8-13  → 配置 7-12   （右腿6关节）
- body 14    → 跳过（fixed）
- body 15-54 → 配置 13-52  （腰/头/双臂/双手）
###### 视觉信息说明：
- 相机挂载点： 通常位于 head_pitch (Body 18) 的末端。
- 手眼协调 (Hand-Eye Calibration)： 如果你要做视觉引导的抓取或交互，你需要计算从 head_pitch 坐标系到 right_wrist_yaw 坐标系的变换矩阵。
- 交互感知： 在手势舞任务中，视觉通常用于检测人脸或人的手势，从而触发机器人的应答动作（例如人出“剪刀”，机器人出“石头”）

## 草稿
手势：比心、数字、打招呼

## 给ai的
### 课题
课题3：手势舞 项目背景 人形机器人不仅需要完成抓取任务，还需具备与人交互的能力。通过 控制5自由度欠驱动手，可以手势动作，如石头、剪刀、布或字母手势 。尽管欠驱动手无法独立控制每根手指，但通过合理设计手部开合程 度和手指耦合运动，仍可实现有区分度的手势。7自由度手臂可将手部 移动到不同空间位置，增强表现力。 任务目标 1. 建立7自由度手臂与5自由度欠驱动手的运动学模型 2. 分析欠驱动手的手指耦合机制，设计至少3种可区分的手势 3. 实现关节空间轨迹规划，使手势动作平滑连贯 4. 将手部移动到不少于2个不同的空间位置，展示相同手势。

我们采用matlab作为唯一软件，7自由度手臂没提到就是不冗余

这是xx文件的信息，还需要提供什么
### 一些话
这是我们机器人建模与控制4人小组准备做的课题，请提供二十天（甚至更少）完成这个项目的步骤；以及我是组长，教我怎么使用github建立它的仓库
### robot.showdetails()的结果：
details
--------------------
Robot: (54 bodies)

 Idx                          Body Name                         Joint Name                         Joint Type                          Parent Name(Idx)   Children Name(s)
 ---                          ---------                         ----------                         ----------                          ----------------   ----------------
   1                      left_hip_roll                      left_hip_roll                           revolute                              base_link(0)   left_hip_yaw(2)  
   2                       left_hip_yaw                       left_hip_yaw                           revolute                          left_hip_roll(1)   left_hip_pitch(3)  
   3                     left_hip_pitch                     left_hip_pitch                           revolute                           left_hip_yaw(2)   left_knee_pitch(4)  
   4                    left_knee_pitch                    left_knee_pitch                           revolute                         left_hip_pitch(3)   left_ankle_pitch(5)  
   5                   left_ankle_pitch                   left_ankle_pitch                           revolute                        left_knee_pitch(4)   left_ankle_roll(6)  
   6                    left_ankle_roll                    left_ankle_roll                           revolute                       left_ankle_pitch(5)   llegsite(7)  
   7                           llegsite                     llegsite_joint                              fixed                        left_ankle_roll(6)   
   8                     right_hip_roll                     right_hip_roll                           revolute                              base_link(0)   right_hip_yaw(9)  
   9                      right_hip_yaw                      right_hip_yaw                           revolute                         right_hip_roll(8)   right_hip_pitch(10)  
  10                    right_hip_pitch                    right_hip_pitch                           revolute                          right_hip_yaw(9)   right_knee_pitch(11)  
  11                   right_knee_pitch                   right_knee_pitch                           revolute                       right_hip_pitch(10)   right_ankle_pitch(12)  
  12                  right_ankle_pitch                  right_ankle_pitch                           revolute                      right_knee_pitch(11)   right_ankle_roll(13)  
  13                   right_ankle_roll                   right_ankle_roll                           revolute                     right_ankle_pitch(12)   rlegsite(14)  
  14                           rlegsite                     rlegsite_joint                              fixed                      right_ankle_roll(13)   
  15                         waist_roll                         waist_roll                           revolute                              base_link(0)   waist_yaw(16)  
  16                          waist_yaw                          waist_yaw                           revolute                            waist_roll(15)   head_yaw(17)  left_shoulder_pitch(19)  right_shoulder_pitch(37)  
  17                           head_yaw                           head_yaw                           revolute                             waist_yaw(16)   head_pitch(18)  
  18                         head_pitch                         head_pitch                           revolute                              head_yaw(17)   
  19                left_shoulder_pitch                left_shoulder_pitch                           revolute                             waist_yaw(16)   left_shoulder_roll(20)  
  20                 left_shoulder_roll                 left_shoulder_roll                           revolute                   left_shoulder_pitch(19)   left_shoulder_yaw(21)  
  21                  left_shoulder_yaw                  left_shoulder_yaw                           revolute                    left_shoulder_roll(20)   left_elbow_pitch(22)  
  22                   left_elbow_pitch                   left_elbow_pitch                           revolute                     left_shoulder_yaw(21)   left_elbow_yaw(23)  
  23                     left_elbow_yaw                     left_elbow_yaw                           revolute                      left_elbow_pitch(22)   left_wrist_pitch(24)  
  24                   left_wrist_pitch                   left_wrist_pitch                           revolute                        left_elbow_yaw(23)   left_wrist_yaw(25)  
  25                     left_wrist_yaw                     left_wrist_yaw                           revolute                      left_wrist_pitch(24)   left_index_finger_2_pitch(26)  left_little_finger_2_pitch(28)  left_middle_finger_2_pitch(30)  left_ring_finger_2_pitch(32)  left_thumb_1_roll(34)  
  26          left_index_finger_2_pitch          left_index_finger_2_pitch                           revolute                        left_wrist_yaw(25)   left_index_finger_3_pitch(27)  
  27          left_index_finger_3_pitch          left_index_finger_3_pitch                           revolute             left_index_finger_2_pitch(26)   
  28         left_little_finger_2_pitch         left_little_finger_2_pitch                           revolute                        left_wrist_yaw(25)   left_little_finger_3_pitch(29)  
  29         left_little_finger_3_pitch         left_little_finger_3_pitch                           revolute            left_little_finger_2_pitch(28)   
  30         left_middle_finger_2_pitch         left_middle_finger_2_pitch                           revolute                        left_wrist_yaw(25)   left_middle_finger_3_pitch(31)  
  31         left_middle_finger_3_pitch         left_middle_finger_3_pitch                           revolute            left_middle_finger_2_pitch(30)   
  32           left_ring_finger_2_pitch           left_ring_finger_2_pitch                           revolute                        left_wrist_yaw(25)   left_ring_finger_3_pitch(33)  
  33           left_ring_finger_3_pitch           left_ring_finger_3_pitch                           revolute              left_ring_finger_2_pitch(32)   
  34                  left_thumb_1_roll                  left_thumb_1_roll                           revolute                        left_wrist_yaw(25)   left_thumb_2_pitch(35)  
  35                 left_thumb_2_pitch                 left_thumb_2_pitch                           revolute                     left_thumb_1_roll(34)   left_thumb_3_pitch(36)  
  36                 left_thumb_3_pitch                 left_thumb_3_pitch                           revolute                    left_thumb_2_pitch(35)   
  37               right_shoulder_pitch               right_shoulder_pitch                           revolute                             waist_yaw(16)   right_shoulder_roll(38)  
  38                right_shoulder_roll                right_shoulder_roll                           revolute                  right_shoulder_pitch(37)   right_shoulder_yaw(39)  
  39                 right_shoulder_yaw                 right_shoulder_yaw                           revolute                   right_shoulder_roll(38)   right_elbow_pitch(40)  
  40                  right_elbow_pitch                  right_elbow_pitch                           revolute                    right_shoulder_yaw(39)   right_elbow_yaw(41)  
  41                    right_elbow_yaw                    right_elbow_yaw                           revolute                     right_elbow_pitch(40)   right_wrist_pitch(42)  
  42                  right_wrist_pitch                  right_wrist_pitch                           revolute                       right_elbow_yaw(41)   right_wrist_yaw(43)  
  43                    right_wrist_yaw                    right_wrist_yaw                           revolute                     right_wrist_pitch(42)   right_index_finger_2_pitch(44)  right_little_finger_2_pitch(46)  right_middle_finger_2_pitch(48)  right_ring_finger_2_pitch(50)  right_thumb_1_roll(52)  
  44         right_index_finger_2_pitch         right_index_finger_2_pitch                           revolute                       right_wrist_yaw(43)   right_index_finger_3_pitch(45)  
  45         right_index_finger_3_pitch         right_index_finger_3_pitch                           revolute            right_index_finger_2_pitch(44)   
  46        right_little_finger_2_pitch        right_little_finger_2_pitch                           revolute                       right_wrist_yaw(43)   right_little_finger_3_pitch(47)  
  47        right_little_finger_3_pitch        right_little_finger_3_pitch                           revolute           right_little_finger_2_pitch(46)   
  48        right_middle_finger_2_pitch        right_middle_finger_2_pitch                           revolute                       right_wrist_yaw(43)   right_middle_finger_3_pitch(49)  
  49        right_middle_finger_3_pitch        right_middle_finger_3_pitch                           revolute           right_middle_finger_2_pitch(48)   
  50          right_ring_finger_2_pitch          right_ring_finger_2_pitch                           revolute                       right_wrist_yaw(43)   right_ring_finger_3_pitch(51)  
  51          right_ring_finger_3_pitch          right_ring_finger_3_pitch                           revolute             right_ring_finger_2_pitch(50)   
  52                 right_thumb_1_roll                 right_thumb_1_roll                           revolute                       right_wrist_yaw(43)   right_thumb_2_pitch(53)  
  53                right_thumb_2_pitch                right_thumb_2_pitch                           revolute                    right_thumb_1_roll(52)   right_thumb_3_pitch(54)  
  54                right_thumb_3_pitch                right_thumb_3_pitch                           revolute                   right_thumb_2_pitch(53) 
