# 机器人手势交互系统

## 文件结构
```
robot_project/
├── main.m                  ← 唯一入口，直接运行这个
├── kepler_WeidJoint/       ← 老师发的一整个文件夹
│   ├── JingChuJointR.urdf
│   ├── meshes/
│   └── ...
├── core/
│   ├── init_robot.m        ← 机器人模型 + 手势库 + 手臂姿态
│   ├── init_vision.m       ← 摄像头 + 检测器 + DL模型加载
│   ├── robot_utils.m       ← 轨迹生成、渲染等工具（classdef）
│   └── vision_utils.m      ← 人脸追踪、手势识别、画面显示（classdef）
├── modes/
│   ├── mode_demo.m         ← 功能1：0-9顺序演示
│   ├── mode_mirror.m       ← 功能2：镜像识别
│   └── mode_rps.m          ← 功能3：猜拳游戏
└── gesture_net.mat         ← 训练好的DL模型（可选）
```

## 运行前准备
1. 将 `JingChuJointR.urdf` 放在 `robot_project/` 目录下
2. （可选）运行 `collect_gesture_data.m` + `train_gesture_net.m` 训练DL模型
   - 无模型时自动降级为传统CV方案（准确率较低）

## 操作说明
| 按键 | 功能 |
|------|------|
| `1`  | 演示模式：0→9手势顺序展示（0-4高位，5-9低位）|
| `2`  | 镜像模式：识别你的手势(0-9)，机器人做相同手势 |
| `3`  | 猜拳模式：出✊✌🖐，机器人出必胜手势 |
| `ESC`| 退出当前模式，回到待机 |
| `Q`  | 退出程序 |

## 猜拳规则
| 你出 | 机器人出 | 结果 |
|------|----------|------|
| 石头(0/✊) | 布(5/🖐) | 机器人赢 |
| 剪刀(2/✌) | 石头(0/✊) | 机器人赢 |
| 布(5/🖐)  | 剪刀(2/✌) | 机器人赢 |

## 手势与数字对应
功能2和功能3用同一套识别，模式决定行为：
- 功能2：识别到"2" → 做手势2
- 功能3：识别到"2"（剪刀）→ 做石头

## 实机部署
- 摄像头编号：`init_vision.m` 第8行 `webcam(1)` 改为机器人摄像头编号
- 音频输出：机器人通过蓝牙5.2连接音箱，MATLAB用 `audioplayer` 播放
- Jetson推理：用 `exportONNXNetwork` 导出模型部署到Jetson Orin NX

# 一、初学者操作指南

## 核心概念：两个最重要的函数

### ① `makeHand` — 控制手指姿态

```matlab
g_1 = makeHand(0, BEND, BEND, BEND, T_IN, HALF, COUPLE);
%              食  小    中    无名  拇展  拇近  耦合比
```

**7 个参数**对应手的 7 个控制量，值只用记 **三个常量**：

| 常量 | 含义 |
|:----:|:----:|
| `0` | 伸直 |
| `BEND` | 完全弯曲（80°） |
| `HALF` | 半弯（45°） |

> 💡 **示例**：想让食指伸直、其余握拳，就把食指位置填 `0`，其余填 `BEND`。

---

### ② `arm = deg2rad([...])` — 控制手臂姿态

```matlab
arm = deg2rad([ 65,  +65,  -10,  60,  30,  30,  0]);
%              肩俯仰 肩外展 肩旋转 肘俯仰 肘旋转 腕俯仰 腕偏航
```

- **7 个数字**对应手臂 7 个关节的角度，单位是**度**
- `deg2rad` 自动将度转换为弧度
- 调整哪个关节就修改对应位置的数字，**正负控制方向**

---

### ③ `buildConfig` — 把手势和手臂姿态组合成完整配置

```matlab
q = buildConfig(q_home, arm, hand, ARM_IDX, HAND_IDX);
```

> 📌 **理解**：把"手臂角度"和"手指角度"拼接在一起，得到机器人的完整关节配置。

**不需要修改这个函数本身。**

---

## 文件职责一览

```
main.m                  唯一入口，直接运行这一个
│
├── core/init_robot.m       手势库在这里，想改手势找这里
├── core/init_vision.m      摄像头参数在这里
├── core/robot_utils.m      轨迹/渲染工具，一般不需要改
├── core/vision_utils.m     手势识别算法，一般不需要改
│
├── modes/mode_demo.m       功能1的逻辑在这里
├── modes/mode_mirror.m     功能2的逻辑在这里
└── modes/mode_rps.m        功能3的逻辑在这里
```

> 🎯 **初学者只需要关注两个文件**：`main.m` 顶部的开关 + `core/init_robot.m` 里的手势定义。

---

## 快速上手步骤

### 第一次运行：

1. 把 `kepler_WeidJoint/` 文件夹放入 `robot_project/`
2. 打开 MATLAB，把当前目录切换到 `robot_project/`
3. 打开 `main.m`，确认第 29 行 `DEBUG_MODE = true`
4. 点击运行，摄像头窗口弹出即为成功

### 调试阶段（轻薄本）：

```matlab
DEBUG_MODE = true;   % 只有摄像头窗口，速度快
```

### 完整演示（游戏本/实机）：

```matlab
DEBUG_MODE = false;  % 摄像头 + 3D机器人双窗口
```

### 三种模式的切换：

| 操作方式 | 操作方法 |
|:--------:|:--------:|
| 电脑键盘 | 点击摄像头窗口后按 `1` / `2` / `3`，`ESC` 退出当前模式 |
| 实物/无键盘 | 对摄像头比 👍 点赞保持 3 秒，自动循环切换 |

---

## 想修改手势怎么做

1. 打开 `core/init_robot.m`，找到手势定义区域
2. 照着格式修改：

```matlab
% 例：把数字1改成中指伸直（而不是食指）
% 参数顺序：食  小    中  无名  拇展  拇近
g_1 = makeHand(BEND, BEND, 0, BEND, T_IN, HALF, COUPLE);
%              ↑握拳        ↑中指伸直
```

> ✅ 改完重新运行 `main.m` 即可生效，**不需要重新训练模型**。

# 二、需要安装的 MATLAB 工具箱

共需要 **6 个工具箱**，在 MATLAB 的 附加功能 里搜索名字安装：

| 工具箱名称 | 用途 | 是否必须 |
|:----------|:-----|:--------:|
| **Robotics System Toolbox** | `importrobot`、`show`、`getTransform`，机器人模型加载和运动学计算的核心 | ✅ 必须 |
| **Computer Vision Toolbox** | `vision.CascadeObjectDetector` 人脸检测、`vision.PointTracker` 人脸追踪、`insertShape` 画框 | ✅ 必须 |
| **Image Processing Toolbox** | `rgb2ycbcr` 肤色分割、`bwconncomp` 连通域、`imresize` 等图像处理函数 | ✅ 必须 |
| **Deep Learning Toolbox** | `trainNetwork` 训练手势识别网络、`classify` 推理、`squeezenet` 预训练模型 | ⚠️ 训练模型时必须，只用传统 CV 可跳过（这里必须） |
| **Deep Learning Toolbox Model for SqueezeNet** | SqueezeNet 预训练权重，迁移学习用 | ⚠️ 和上面配套，一起装（必须） |
| **MATLAB Support Package for USB Webcams** | USB 摄像头驱动支持 | ✅ 必须 |

**（想直接打开.slx文件需要安装simulink）**
