## 1. 右臂 (Right Arm) —— 核心控制对象
### 自由度：7-DOF
任务目标中的主手臂,是一个典型的冗余配置（7个关节控制空间6个维度的位姿）。

| 序号 | 关节名称 | 类型 | 功能描述 |
|:----:|:---------:|:----:|:---------|
| 37 | right_shoulder_pitch | Revolute | 肩部俯仰 |
| 38 | right_shoulder_roll | Revolute | 肩部外展/内收 |
| 39 | right_shoulder_yaw | Revolute | 肩部旋转（大臂旋内/旋外） |
| 40 | right_elbow_pitch | Revolute | 肘部俯仰（屈肘） |
| 41 | right_elbow_yaw | Revolute | 肘部旋转（前臂旋前/旋后） |
| 42 | right_wrist_pitch | Revolute | 腕部俯仰 |
| 43 | right_wrist_yaw | Revolute | 腕部偏航（末端旋转） |

## 2. 右手 (Right Hand) —— 欠驱动手势设计
**结构：11个物理关节，由5个驱动源（电机）耦合控制。需要通过控制电机来带动以下物理关节：**
- 食指 (Index): &nbsp; 44 (2_pitch), &nbsp; 45 (3_pitch) —— 耦合运动
- 小指 (Little): &nbsp; 46 (2_pitch), &nbsp; 47 (3_pitch) —— 耦合运动
- 中指 (Middle): &nbsp; 48 (2_pitch), &nbsp; 49 (3_pitch) —— 耦合运动
- 无名指 (Ring): &nbsp; 50 (2_pitch), &nbsp; 51 (3_pitch) —— 耦合运动
- 大拇指 (Thumb): &nbsp; 52 (1_roll), &nbsp; 53 (2_pitch), &nbsp; 54 (3_pitch) —— 较为复杂的运动
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

###### 视觉信息说明：
- 相机挂载点： 通常位于 head_pitch (Body 18) 的末端。
- 手眼协调 (Hand-Eye Calibration)： 如果你要做视觉引导的抓取或交互，你需要计算从 head_pitch 坐标系到 right_wrist_yaw 坐标系的变换矩阵。
- 交互感知： 在手势舞任务中，视觉通常用于检测人脸或人的手势，从而触发机器人的应答动作（例如人出“剪刀”，机器人出“石头”）
