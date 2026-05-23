% 1. 加载完整机器人并提取右臂
robot = importrobot('JingChuJointR.urdf'); 
rightArm = subtree(robot, 'right_shoulder_pitch'); 

% 核心修改：将右臂模型的数据格式设置为行向量 ('row')
rightArm.DataFormat = 'row'; 

% 2. 创建专门针对右臂的交互界面
gui_rightArm = interactiveRigidBodyTree(rightArm);

% 3. 设置初始姿态（此时 homeConfiguration 返回的是 double 向量，不再报错）
gui_rightArm.Configuration = homeConfiguration(rightArm);

% 4. 设置窗口标题
title('右臂控制面板 - 仅操控右臂/右手');
