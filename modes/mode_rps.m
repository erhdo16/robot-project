% ========================================================================
%  modes/mode_rps.m — 功能3：猜拳游戏模式
%
%  猜拳规则（机器人视角）：
%    人出石头(0) -> 机器人出布(5)   -> 机器人赢
%    人出剪刀(2) -> 机器人出石头(0) -> 机器人赢
%    人出布(5)   -> 机器人出剪刀(2) -> 机器人赢
%
%  修复：
%    - insertText 去掉 emoji，避免字体警告
%    - responded 标志防止重复触发
%    - 归位后增加冷却帧，防止归位过程中再次识别
% ========================================================================
function q_current = mode_rps(R, V, ROS, ax_robot, hImg, q_current, fig_cam)

ru = robot_utils;

RPS = containers.Map(...
    {'0','2','5'}, ...
    { struct('hand',R.g_paper,    'robot_name','Bu',   'human_name','ShiTou','win_txt','Ji qi ren ying! Bu bao shi tou'), ...
      struct('hand',R.g_rock,     'robot_name','ShiTou','human_name','JianDao','win_txt','Ji qi ren ying! Shi tou jian jian dao'), ...
      struct('hand',R.g_scissors, 'robot_name','JianDao','human_name','Bu',  'win_txt','Ji qi ren ying! Jian dao jian bu') });

% 显示用中文（不含emoji，避免insertText字体警告）
RPS_SHOW = containers.Map({'0','2','5'}, ...
    {'石头 vs 布  机器人赢', '剪刀 vs 石头  机器人赢', '布 vs 剪刀  机器人赢'});
HUMAN_SHOW = containers.Map({'0','2','5'}, {'石头','剪刀','布'});
ROBOT_SHOW = containers.Map({'0','2','5'}, {'布','石头','剪刀'});

RPS_KEYS      = {'0','2','5'};
last_label    = '';
stable_count  = 0;
frame_count   = 0;
cached_label  = '';
cached_conf   = 0;
cached_bbox   = [];
responded     = false;
cooldown      = 0;        % 归位后冷却帧数，防止归位中再次触发
COOLDOWN_FRAMES = 20;    % 冷却帧数

score_robot = 0;
score_human = 0;
rounds      = 0;

fprintf('[猜拳模式] 启动！出石头、剪刀或布\n');
fprintf('[猜拳模式] 请将脸移出画面后再出拳\n');
fprintf('[猜拳模式] 按键盘 1/2/3 切换模式\n');

while ishandle(fig_cam) && isequal(getappdata(fig_cam,'mode'),'rps')
    frame_count = frame_count + 1;
    frame = snapshot(V.cam);

    do_detect = (mod(frame_count, V.DETECT_INTERVAL) == 1);
    do_dl     = (mod(frame_count, V.DL_INTERVAL)     == 1);

    [face_visible, V, cached_bbox] = vision_utils.updateFaceTrack(frame, V, do_detect);

    % ── 检测到人脸：暂停识别 ──────────────────────────────────────
    if face_visible
        cached_label = '';
        cached_conf  = 0;
        stable_count = 0;
        last_label   = '';
        responded    = false;
        cooldown     = 0;
        stxt = sprintf('检测到人脸，请移出画面后出拳 | 比分 %d:%d', ...
            score_robot, score_human);
        vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式3: 猜拳');
        drawnow limitrate;
        continue;
    end

    % ── 冷却期：归位完成后等几帧再开始新一局 ─────────────────────
    if cooldown > 0
        cooldown = cooldown - 1;
        stxt = sprintf('准备下一局... | 比分 机器人%d : 你%d', score_robot, score_human);
        vision_utils.updateCamView(hImg, frame, [], stxt, '模式3: 猜拳');
        drawnow limitrate;
        continue;
    end

    % ── 正常识别 ──────────────────────────────────────────────────
    if do_dl
        [cached_label, cached_conf] = vision_utils.detectGesture(frame, V, []);
    end

    is_rps = ismember(cached_label, RPS_KEYS);

    if strcmp(cached_label, last_label)
        stable_count = stable_count + 1;
    else
        stable_count = 1;
        last_label   = cached_label;
        responded    = false;
    end

    conf_ok = (V.use_dl && cached_conf > 0.82) || (~V.use_dl && cached_conf > 0.5);

    if stable_count >= V.STABLE_THRESH && is_rps && conf_ok && ~responded
        responded = true;
        rounds    = rounds + 1;
        score_robot = score_robot + 1;

        win_str  = RPS_SHOW(cached_label);
        hstr     = HUMAN_SHOW(cached_label);
        rstr     = ROBOT_SHOW(cached_label);
        fprintf('[猜拳] 第%d局 | 人: %s  机器人: %s | %s\n', rounds, hstr, rstr, win_str);

        info  = RPS(cached_label);
        q_new = ru.buildConfig(R.q_home, R.arm_lo, info.hand, R.ARM_IDX, R.HAND_IDX);
        traj  = ru.smoothTraj(q_current, q_new, R.TRAJ_N_FAST);
        q_current = ru.execTraj(traj, R.robot, ax_robot, ...
            win_str, V.cam, hImg, 2);

        if ~isempty(ROS)
            ros2_utils.sendRPS(ROS, cached_label);
        end

        % 结果画面（纯中文，无 emoji）
        result_frame = insertText(frame, [20 20], win_str, ...
            'FontSize', 24, 'TextColor', 'yellow', ...
            'BoxColor', [0.1 0.1 0.1], 'BoxOpacity', 0.7, ...
            'Font', 'Microsoft YaHei');
        score_str = sprintf('比分  机器人:%d  你:%d  共%d局', score_robot, score_human, rounds);
        result_frame = insertText(result_frame, [20 70], score_str, ...
            'FontSize', 18, 'TextColor', 'white', ...
            'BoxColor', 'black', 'BoxOpacity', 0.6, ...
            'Font', 'Microsoft YaHei');
        set(hImg, 'CData', result_frame); drawnow;
        pause(2.0);

        % 归位
        traj = ru.smoothTraj(q_current, R.q_home, R.TRAJ_N_MID);
        q_current = ru.execTraj(traj, R.robot, ax_robot, ...
            '准备下一局', V.cam, hImg, 2);

        % 归位后强制冷却，清空所有状态
        stable_count = 0;
        last_label   = '';
        cached_label = '';
        cached_conf  = 0;
        responded    = false;
        cooldown     = COOLDOWN_FRAMES;
        continue;
    end

    % ── 摄像头显示 ────────────────────────────────────────────────
    if is_rps && ~isempty(cached_label)
        stxt = sprintf('检测到[%s] 稳定:%d/%d | 等待出拳...', ...
            cached_label, min(stable_count, V.STABLE_THRESH), V.STABLE_THRESH);
    else
        stxt = sprintf('猜拳模式 | 出 0石头 2剪刀 5布 | 比分 %d:%d', ...
            score_robot, score_human);
    end
    vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式3: 猜拳');
    drawnow limitrate;
end

fprintf('[猜拳模式] 结束 | 最终比分 机器人:%d 你:%d\n', score_robot, score_human);
end
