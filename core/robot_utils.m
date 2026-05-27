% ========================================================================
%  core/robot_utils.m
%  轨迹生成、配置构建、机器人渲染等通用工具
%  全部为静态函数，通过 robot_utils.xxx() 调用
%  用法示例：
%    traj = robot_utils.smoothTraj(q1, q2, 60);
%    q    = robot_utils.buildConfig(q_home, arm, hand, ARM_IDX, HAND_IDX);
% ========================================================================
classdef robot_utils
methods(Static)

    % ── 五次多项式平滑轨迹 ─────────────────────────────────────────
    function traj = smoothTraj(q_start, q_end, n)
        t = linspace(0,1,n);
        s = 6*t.^5 - 15*t.^4 + 10*t.^3;
        traj = repmat(q_start,n,1) + s'*(q_end - q_start);
    end

    % ── 构建完整关节配置 ───────────────────────────────────────────
    function q = buildConfig(q_base, arm7, hand11, aidx, hidx)
        q = q_base;
        q(aidx) = arm7;
        q(hidx) = hand11;
    end

    % ── 执行轨迹（含摄像头帧更新，不阻塞画面）────────────────────
    %    cam       : webcam对象（可为[]，则不抓帧）
    %    hImg      : imshow句柄（可为[]）
    %    ax_robot  : 机器人axes句柄
    %    skip      : 每skip帧渲染一次机器人
    function q_end = execTraj(traj, robot, ax_robot, titleStr, ...
                              cam, hImg, skip)
        if nargin < 7, skip = 2; end
        q_prev = traj(1,:);
        TOL    = 1e-3;
        for f = 1:size(traj,1)
            q = traj(f,:);
            % 摄像头更新
            if ~isempty(cam) && ~isempty(hImg)
                try
                    set(hImg,'CData',snapshot(cam));
                catch; end
            end
            % 机器人渲染（变化够大才重绘）
            if (mod(f,skip)==0 || f==size(traj,1)) && ...
               max(abs(q - q_prev)) > TOL
                robot_utils.showRobot(robot, q, ax_robot, titleStr);
                q_prev = q;
            end
            drawnow limitrate;
        end
        q_end = traj(end,:);
    end

    % ── 机器人3D渲染（ax为空时直接跳过，用于调试模式）──────────────
    function showRobot(robot, q, ax, titleStr)
        if isempty(ax), return; end   % DEBUG_MODE=true时ax_robot=[]
        show(robot, q, 'Parent',ax, 'PreservePlot',false, 'Frames','off');
        title(ax, titleStr, 'FontSize',12, 'FontWeight','bold');
        view(ax, 135, 15);
        axis(ax, [-0.4, 0.8, -0.6, 0.4, -0.2, 1.3]);
    end

    % ── 时间戳字符串 ───────────────────────────────────────────────
    function s = ts()
        s = datestr(now,'HH:MM:SS');
    end

end
end