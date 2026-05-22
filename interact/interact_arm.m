robot = importrobot('JingChuJointR.urdf'); % 加载完整机器人
rightArm = subtree(robot, 'right_shoulder_pitch'); % 提取右臂

% 2. 创建专门针对右臂的交互界面
gui_rightArm = interactiveRigidBodyTree(rightArm);

% 3. 设置窗口标题和初始姿态
gui_rightArm.CurrentRobotConfiguration = homeConfiguration(rightArm);
title('右臂控制面板 - 仅操控右臂/右手');
