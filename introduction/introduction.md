## 1. 右臂 (Right Arm) —— 核心控制对象
### 自由度：7-DOF
任务目标中的主手臂,是一个典型的冗余配置（7个关节控制空间6个维度的位姿）。
**序号	关节名称	类型	功能描述**

37	right_shoulder_pitch	Revolute	肩部俯仰

38	right_shoulder_roll	Revolute	肩部外展/内收

39	right_shoulder_yaw	Revolute	肩部旋转（大臂旋内/旋外）

40	right_elbow_pitch	Revolute	肘部俯仰（屈肘）

41	right_elbow_yaw	Revolute	肘部旋转（前臂旋前/旋后）

42	right_wrist_pitch	Revolute	腕部俯仰

43	right_wrist_yaw	Revolute	腕部偏航（末端旋转）
## 2. 右手 (Right Hand) —— 欠驱动手势设计
**结构：11个物理关节，由5个驱动源（电机）耦合控制。需要通过控制电机来带动以下物理关节：**
