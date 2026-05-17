## 1. 右臂 (Right Arm) —— 核心控制对象
### 自由度：7-DOF
任务目标中的主手臂,是一个典型的冗余配置（7个关节控制空间6个维度的位姿）。

| 序号 | 关节名称 | 类型 | 功能描述 |
|:----:|:---------:|:----:|:---------|
| 37 | right_shoulder_pitch | Revolute | 肩部俯仰 前((+)后(-)抬臂 |
| 38 | right_shoulder_roll | Revolute | 肩部外展(+)/内收(-) |
| 39 | right_shoulder_yaw | Revolute | 肩部旋转（大臂旋内+/旋外-） |
| 40 | right_elbow_pitch | Revolute | 肘部俯仰 (屈肘+/伸展-) |
| 41 | right_elbow_yaw | Revolute | 肘部旋转（前臂旋前(内)+/旋后-） |
| 42 | right_wrist_pitch | Revolute | 腕部俯仰(勾手+/翘手-)  |
| 43 | right_wrist_yaw | Revolute | 腕部偏航（末端旋转） (向拇指方向偏+|
 
 Yaw-Pitch-Roll
- body 15-54 → 配置 13-52  （腰/头/双臂/双手）
