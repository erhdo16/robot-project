clear; clc; close all;

% 导入机器人模型
robot = importrobot('JingChuJointR.urdf');
robot.DataFormat = 'row';
q_home = homeConfiguration(robot);

% 定义关节索引
ARM_IDX = (37:43) - 2;
HAND_IDX = (44:54) - 2;

% 定义手势构造函数
function h = makeHand(idx, lit, mid, rng, th_r, th_p, c)
    h = zeros(1,11);
    h(1) = idx;
    h(2) = c * idx;
    h(3) = lit;
    h(4) = c * lit;
    h(5) = mid;
    h(6) = c * mid;
    h(7) = rng;
    h(8) = c * rng;
    h(9) = th_r;
    h(10) = th_p;
    h(11) = c * th_p;
end

% 定义配置构建函数
function q = buildConfig(q_base, arm7, hand11, aidx, hidx)
    q = q_base;
    q(aidx) = arm7;
    q(hidx) = hand11;
end

% ======================================================
%  关节角常量
%    COUPLE : 耦合比（远端 = COUPLE * 近端）
%    BEND   : 完全弯曲 (~80°)
%    HALF   : 半弯    (~45°)
%    QUAT   : 四分之一弯 (~20°)
%    T_OUT  : 拇指外展  (+35°)
%    T_IN   : 拇指内收  (-20°)
%    T_PINCH: 拇指对捏近端 (~40°)
% ======================================================
COUPLE  = 0.7;
BEND    = deg2rad(80);
HALF    = deg2rad(45);
QUAT    = deg2rad(20);
T_OUT   = deg2rad(35);
T_IN    = deg2rad(-20);
T_PINCH = deg2rad(40);   % 拇指对捏时的近端弯曲角

#各种手势的代码
#g_0 = makeHand(BEND, BEND, BEND, BEND,  T_IN, HALF, COUPLE);
#g_1 = makeHand(0, BEND, BEND, BEND, T_IN, HALF, COUPLE);
#g_2 = makeHand(0, BEND, 0, BEND, T_IN,  HALF, COUPLE);
#g_3 = makeHand(0, BEND, 0, BEND, T_OUT, 0, COUPLE);
#g_4 = makeHand(0, 0, 0, 0, T_IN, BEND, COUPLE);
#g_5 = makeHand(0,    0,    0,    0,     T_OUT, 0,       COUPLE);
#g_6 = makeHand(BEND, 0,    BEND, BEND,  T_OUT, 0,       COUPLE);
#g_7 = makeHand(0, 0, BEND, BEND,  T_IN, 0,    COUPLE);
#g_8 = makeHand(0,    BEND, BEND, BEND,  T_OUT, 0,       COUPLE);
#g_9 = makeHand(QUAT, BEND, BEND, BEND,  T_IN, HALF,    COUPLE);

% 定义手臂姿态（您提供的角度）
arm = deg2rad([65, +65, -10, 60, 30, 30, 0]); #展示手势的手臂姿态

% 构建完整配置
q_final = buildConfig(q_home, arm, g_paper, ARM_IDX, HAND_IDX);

% 显示最终姿态
figure('Name','机器人最终姿态','Position',[300 200 900 700]);
show(robot, q_final, 'Frames', 'off', 'Visuals', 'on');
view(135, 20);
title('手臂姿态: 肩俯仰65° | 肩外展65° | 肘俯仰60° | 肘旋转30°', ...
      'FontSize', 12, 'FontWeight', 'bold');
axis([-0.5, 0.8, -0.9, 0.6, -0.2, 1.2]);
grid on;

% 输出末端位置信息
T_end = getTransform(robot, q_final, 'right_wrist_yaw');
fprintf('\n========== 姿态信息 ==========\n');
fprintf('肩俯仰: 65°\n');
fprintf('肩外展: 60°\n');
fprintf('肩旋转: -10°\n');
fprintf('肘俯仰: 60°\n');
fprintf('肘旋转: 30°\n');
fprintf('腕俯仰: 30°\n');
fprintf('腕偏航: 0°\n');
fprintf('\n右手腕末端坐标:\n');
fprintf('  X: %.3f m (左右方向)\n', T_end(1,4));
fprintf('  Y: %.3f m (前后方向)\n', T_end(2,4));
fprintf('  Z: %.3f m (上下方向)\n', T_end(3,4));
fprintf('================================\n');
