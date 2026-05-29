% ========================================================================
%  modes/mode_demo.m — 功能1：演示模式
%
%  机器人按顺序展示数字手势 0→9：
%    0-4 用高位手臂姿态（arm_hi）
%    5-9 用低位手臂姿态（arm_lo）
%  每个手势停留 R.HOLD_SECS 秒，切换间用平滑轨迹过渡
%
%  调用方式：
%    q_current = mode_demo(R, ax_robot, cam, hImg, q_current)
%  返回执行结束时的关节角
% ========================================================================
function q_current = mode_demo(R, ax_robot, cam, hImg, q_current)

ru = robot_utils;   % 工具类别名

% 手势列表（按0~9顺序）
gestures = {R.g_0,R.g_1,R.g_2,R.g_3,R.g_4, ...
            R.g_5,R.g_6,R.g_7,R.g_8,R.g_9};
labels   = {'0','1','2','3','4','5','6','7','8','9'};

% 前5个高位，后5个低位
arm_for  = [repmat({R.arm_hi},1,5), repmat({R.arm_lo},1,5)];

% 进入时立刻刷新摄像头标题，清除上一个模式的残留
if ~isempty(cam) && ~isempty(hImg)
    try
        vision_utils.updateCamView(hImg, snapshot(cam), [], ...
            '演示模式 | 0→9 手势演示中...', '模式1: 演示');
    catch; end
end

% 高→低位切换时用慢速轨迹
fprintf('[演示模式] 开始 0→9 手势演示\n');

for k = 1:10
    arm   = arm_for{k};
    hand  = gestures{k};
    label = labels{k};

    q_target = ru.buildConfig(R.q_home, arm, hand, R.ARM_IDX, R.HAND_IDX);

    % 切换轨迹速度：跨姿态（4→5）慢，同姿态内中速
    if k == 6   % 高位→低位切换点
        N = R.TRAJ_N_SLOW;
    else
        N = R.TRAJ_N_MID;
    end

    traj = ru.smoothTraj(q_current, q_target, N);
    q_current = ru.execTraj(traj, R.robot, ax_robot, ...
        ['演示模式  手势 [' label ']'], cam, hImg, 2);

    fprintf('[演示模式] 手势 [%s] 到位\n', label);

    % 停留（同时保持摄像头刷新）
    t0 = tic;
    while toc(t0) < R.HOLD_SECS
        if ~isempty(cam) && ~isempty(hImg)
            try, set(hImg,'CData',snapshot(cam)); catch; end
        end
        drawnow limitrate;
        pause(0.03);
    end
end

fprintf('[演示模式] 演示完成，归零\n');
traj = ru.smoothTraj(q_current, R.q_home, R.TRAJ_N_SLOW);
q_current = ru.execTraj(traj, R.robot, ax_robot, '演示完成，归零', cam, hImg, 2);

end
