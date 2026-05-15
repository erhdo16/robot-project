robot = importrobot('JingChuJointR.urdf'); % 加载右臂模型
show(robot);
showdetails(robot); % 查看关节名称（Joints）和层级关系
%%
% 1. 提取右臂子树
rightArm = subtree(robot, 'right_shoulder_pitch');

% 2. 查看这个新模型
showdetails(rightArm) 

% 3. 可视化一下，看看是不是只剩下一只手了
show(rightArm);
