% 1. 准备右臂模型
robot = importrobot('JingChuJointR.urdf');
rightArm = subtree(robot, 'right_shoulder_pitch');

% 2. 获取关节数量和范围
numJoints = numel(homeConfiguration(rightArm));
jointNames = string({rightArm.Bodies{1:numJoints}.Name});

% 3. 创建自定义GUI（使用uifigure）
fig = uifigure('Name', '右臂运动控制面板', 'Position', [100 100 1200 600]);

% 创建网格布局
grid = uigridlayout(fig, [1 2], 'ColumnWidth', {'1x', '2x'});

% 左侧：控制面板
controlPanel = uigridlayout(grid, [numJoints+2, 1], 'Padding', [10 10 10 10]);
uilabel(controlPanel, 'Text', '右臂关节控制滑块', 'FontSize', 14, 'FontWeight', 'bold');

% 存储滑块和标签的句柄
sliders = gobjects(numJoints, 1);
valueLabels = gobjects(numJoints, 1);

% 获取关节极限
jointLimits = getTransform(rightArm, homeConfiguration(rightArm), 'right_wrist_yaw'); % 需要正确获取

for i = 1:numJoints
    % 每个关节的容器
    panel = uigridlayout(controlPanel, [1 3], 'ColumnWidth', {'1.5x', '3x', '0.8x'});
    
    % 关节名称标签
    uilabel(panel, 'Text', char(jointNames(i)), 'HorizontalAlignment', 'right');
    
    % 滑块（示例范围 -pi 到 pi）
    sld = uislider(panel, 'Limits', [-pi, pi], 'Value', 0);
    sliders(i) = sld;
    
    % 数值显示标签
    lbl = uilabel(panel, 'Text', '0.00 rad', 'HorizontalAlignment', 'left');
    valueLabels(i) = lbl;
    
    % 添加回调
    addlistener(sld, 'Value', 'PostSet', @(src, event) updateRobot());
end

% 重置按钮
resetBtn = uibutton(controlPanel, 'Text', '重置所有关节', 'ButtonPushedFcn', @(btn, event) resetJoints());

% 右侧：3D显示区域
rightPanel = uigridlayout(grid);
ax = uiaxes(rightPanel);
view(ax, 3);
axis(ax, 'equal');
title(ax, '右臂实时姿态');

% 初始化配置
currentConfig = homeConfiguration(rightArm);
show(rightArm, currentConfig, 'Parent', ax);

% 更新机器人的函数
    function updateRobot()
        % 从滑块读取值
        for i = 1:numJoints
            currentConfig(i).JointPosition = sliders(i).Value;
            valueLabels(i).Text = sprintf('%.2f rad', sliders(i).Value);
        end
        % 刷新显示
        show(rightArm, currentConfig, 'Parent', ax);
        title(ax, sprintf('右臂实时姿态 | 更新于: %s', datestr(now, 'HH:MM:SS')));
    end

% 重置函数
    function resetJoints()
        homeConfig = homeConfiguration(rightArm);
        for i = 1:numJoints
            sliders(i).Value = homeConfig(i).JointPosition;
        end
        updateRobot();
    end

% 初始更新
updateRobot();
