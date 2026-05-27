% ========================================================================
%  modes/mode_rps.m — 功能3：猜拳游戏模式
%
%  识别人的"剪刀/石头/布"（对应手势2/0/5），
%  机器人出能赢的手势，并显示胜负结果
%
%  猜拳规则（机器人视角）：
%    人出石头(0) → 机器人出布(5)    → 机器人赢
%    人出剪刀(2) → 机器人出石头(0)  → 机器人赢
%    人出布(5)   → 机器人出剪刀(2)  → 机器人赢
%
%  流程：
%    1. 等待人出手势（稳定N帧）
%    2. 机器人做出对应手势 + 打印结果
%    3. 停留2秒后归位，等待下一局
% ========================================================================
function q_current = mode_rps(R, V, ax_robot, hImg, q_current, fig_cam)

ru = robot_utils;

% ── 猜拳逻辑表 ────────────────────────────────────────────────────────
%  key   = 人出的手势标签（只响应 '0','2','5'）
%  hand  = 机器人应出的手部向量
%  name_human  = 人的手势名
%  name_robot  = 机器人手势名
%  result_txt  = 显示文字

RPS = struct();
RPS('0') = struct('hand',R.g_paper,    'robot_name','布🖐',   ...
                  'human_name','石头✊','win_txt','机器人赢！布包石头');
RPS('2') = struct('hand',R.g_rock,     'robot_name','石头✊', ...
                  'human_name','剪刀✌','win_txt','机器人赢！石头剪剪刀');
RPS('5') = struct('hand',R.g_scissors, 'robot_name','剪刀✌', ...
                  'human_name','布🖐',  'win_txt','机器人赢！剪刀剪布');

% 注意：g_rock = g_0，g_scissors = g_2（你的代码里已区分）
% g_paper 是专用的布手势（拇指不外展），和 g_5 略有区别

RPS_KEYS      = {'0','2','5'};
last_label    = '';
stable_count  = 0;
frame_count   = 0;
cached_label  = '';
cached_conf   = 0;
cached_bbox   = [];

% 分数统计
score_robot = 0;
score_human = 0;
rounds      = 0;

fprintf('[猜拳模式] 启动！出石头✊、剪刀✌或布🖐\n');
fprintf('[猜拳模式] 按键盘 1/2/3 切换模式\n');

while ishandle(fig_cam) && ~isequal(getappdata(fig_cam,'mode'),'switch')
    frame_count = frame_count + 1;
    frame = snapshot(V.cam);

    do_detect = (mod(frame_count, V.DETECT_INTERVAL) == 1);
    do_dl     = (mod(frame_count, V.DL_INTERVAL)     == 1);

    [~, V, cached_bbox] = vision_utils.updateFaceTrack(frame, V, do_detect);

    if do_dl
        [cached_label, cached_conf] = vision_utils.detectGesture(frame, V);
    end

    % 只对猜拳手势响应（过滤1/3/4/6/7/8/9）
    is_rps = ismember(cached_label, RPS_KEYS);

    if strcmp(cached_label, last_label)
        stable_count = stable_count + 1;
    else
        stable_count = 1;
        last_label   = cached_label;
    end

    conf_ok = (V.use_dl && cached_conf > 0.75) || (~V.use_dl && cached_conf > 0.5);

    if stable_count == V.STABLE_THRESH && is_rps && conf_ok
        info    = RPS(cached_label);
        rounds  = rounds + 1;
        score_robot = score_robot + 1;  % 机器人永远赢（出必胜手势）

        fprintf('[猜拳] 第%d局 | 人: %s → 机器人: %s | %s\n', ...
            rounds, info.human_name, info.robot_name, info.win_txt);

        % 做出必胜手势
        q_new = ru.buildConfig(R.q_home, R.arm_lo, info.hand, R.ARM_IDX, R.HAND_IDX);
        traj  = ru.smoothTraj(q_current, q_new, R.TRAJ_N_FAST);
        q_current = ru.execTraj(traj, R.robot, ax_robot, ...
            [info.win_txt '  机器人: ' info.robot_name], V.cam, hImg, 2);

        % 显示胜负，停留2秒
        win_frame = insertText(frame, [20 20], info.win_txt, ...
            'FontSize',28,'TextColor','yellow','BoxColor',[0.1 0.1 0.1],...
            'BoxOpacity',0.7);
        score_str = sprintf('比分  机器人:%d  你:%d  共%d局', ...
            score_robot, score_human, rounds);
        win_frame = insertText(win_frame,[20 80],score_str,...
            'FontSize',18,'TextColor','white','BoxColor','black','BoxOpacity',0.6);
        set(hImg,'CData',win_frame); drawnow;
        pause(2.0);

        % 归位，准备下一局
        traj = ru.smoothTraj(q_current, R.q_home, R.TRAJ_N_MID);
        q_current = ru.execTraj(traj, R.robot, ax_robot, ...
            '准备下一局...', V.cam, hImg, 2);
        stable_count = 0;
        last_label   = '';
    end

    % 摄像头显示
    if is_rps && ~isempty(cached_label)
        stxt = sprintf('检测到[%s] 稳定:%d/%d | 等待出拳...', ...
            cached_label, min(stable_count,V.STABLE_THRESH), V.STABLE_THRESH);
    else
        stxt = sprintf('猜拳模式 | 出✊✌🖐  比分 %d:%d', score_robot, score_human);
    end
    vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式3: 猜拳');
    drawnow limitrate;
end

fprintf('[猜拳模式] 结束 | 最终比分 机器人:%d 你:%d\n', score_robot, score_human);
end
