% ========================================================================
%  main.m — 主入口
%
%  模式切换方式（两种，同时支持）：
%    ① 键盘（电脑调试用）：点击摄像头窗口后按 1/2/3/Q/ESC
%    ② 手势触发（实物用）：对摄像头比"点赞👍"保持 3 秒
%       → 循环切换 idle→demo→mirror→rps→idle
%
%  打招呼逻辑：
%    - PointTracker追踪，同一张脸只打一次招呼
%    - 打完招呼后手臂回到 q_home（完全初始位置）
%    - 人离开后重置，下次见面重新打招呼
% ========================================================================

clear; clc; close all;

% ========================================================================
%  ★ 调试开关 ★  在这里修改，只需改这一行
%
%  DEBUG_MODE = true   调试模式（轻薄本 / 低配机器）
%    · 关闭3D机器人窗口，只保留摄像头窗口
%    · 所有手势识别、状态机逻辑照常运行
%    · 帧率显著提升，适合在电脑上验证识别效果
%
%  DEBUG_MODE = false  完整模式（游戏本 / 实机 / 组员高配机器）
%    · 双窗口：摄像头 + 3D机器人实时动画
%    · 组员拿到代码后只需把下面这行改成 false
% ========================================================================
DEBUG_MODE = false;   % ← 轻薄本用 true，游戏本/实机改 false

% ── 路径修复：基于main.m自身位置，不依赖MATLAB当前目录 ──────────────────
SCRIPT_DIR = fileparts(mfilename('fullpath'));
addpath(fullfile(SCRIPT_DIR,'core'));
addpath(fullfile(SCRIPT_DIR,'modes'));

% ────────────────────────────────────────────────────────────────────────
%  初始化
% ────────────────────────────────────────────────────────────────────────
fprintf('========================================\n');
fprintf('  机器人手势交互系统 启动中...\n');
fprintf('========================================\n');

R = init_robot();
V = init_vision();

% ────────────────────────────────────────────────────────────────────────
%  手势触发模式切换的参数
%    触发手势：点赞(g_thumb)，识别标签 'thumb'
%    需连续稳定识别 SWITCH_HOLD_FRAMES 帧才切换（约3秒@30fps）
% ────────────────────────────────────────────────────────────────────────
SWITCH_HOLD_FRAMES = 90;    % 约3秒（摄像头约30fps）
switch_count       = 0;     % 当前连续识别到点赞的帧数
MODE_SEQ = {'idle','demo','mirror','rps'};  % 切换顺序

% ────────────────────────────────────────────────────────────────────────
%  图形界面
% ────────────────────────────────────────────────────────────────────────
fig_cam = figure('Name','摄像头  [1]演示 [2]镜像 [3]猜拳 [Q]退出 | 实物:点赞3秒切换', ...
    'Position',[20 260 660 510], ...
    'Renderer','opengl', ...
    'KeyPressFcn', @onKey);
setappdata(fig_cam,'quit', false);
setappdata(fig_cam,'mode', 'idle');

ax_cam = axes(fig_cam,'Position',[0 0 1 1]);
hImg   = imshow(zeros(V.CAM_H,V.CAM_W,3,'uint8'),'Parent',ax_cam);

% 3D机器人窗口（调试模式下关闭以提升帧率）
if ~DEBUG_MODE
    fig_robot = figure('Name','机器人3D', ...
        'Position',[700 260 680 510],'Renderer','opengl');
    ax_robot = axes(fig_robot);
    robot_utils.showRobot(R.robot, R.q_home, ax_robot, '待机');
    drawnow;
else
    fig_robot = [];
    ax_robot  = [];
    fprintf('[调试模式] 3D窗口已关闭，仅运行摄像头视图\n');
end

% ────────────────────────────────────────────────────────────────────────
%  状态变量
% ────────────────────────────────────────────────────────────────────────
q_current    = R.q_home;
greeted      = false;
face_lost_ct = 0;
frame_count  = 0;
CURRENT_MODE = 'idle';

fprintf('\n【电脑调试】键盘操作（需先点击摄像头窗口）:\n');
fprintf('  1 → 演示模式   2 → 镜像模式   3 → 猜拳模式\n');
fprintf('  ESC → 退出当前模式   Q → 退出程序\n');
fprintf('\n【实物操作】对摄像头比 👍 点赞手势保持约3秒 → 循环切换模式\n\n');

% ────────────────────────────────────────────────────────────────────────
%  主循环
% ────────────────────────────────────────────────────────────────────────
while ishandle(fig_cam) && ~getappdata(fig_cam,'quit')

    frame_count = frame_count + 1;
    frame       = snapshot(V.cam);
    do_detect   = (mod(frame_count, V.DETECT_INTERVAL) == 1);
    do_dl       = (mod(frame_count, V.DL_INTERVAL)     == 1);

    % ── 人脸追踪 ────────────────────────────────────────────────────
    [face_stable, V, bbox] = vision_utils.updateFaceTrack(frame, V, do_detect);

    if face_stable
        face_lost_ct = 0;
    elseif do_detect
        face_lost_ct = face_lost_ct + 1;
    end

    % 人离开 → 归零 + 重置招呼标志
    if face_lost_ct >= V.FACE_LOST_MAX && greeted
        greeted      = false;
        face_lost_ct = 0;
        switch_count = 0;
        fprintf('[%s] 人脸消失 → 归零\n', robot_utils.ts());
        traj = robot_utils.smoothTraj(q_current, R.q_home, R.TRAJ_N_SLOW);
        q_current = robot_utils.execTraj(traj, R.robot, ax_robot, ...
            '人离开，归零', V.cam, hImg, 2);
        CURRENT_MODE = 'idle';
        setappdata(fig_cam,'mode','idle');
    end

    % ── 打招呼（同一张脸只触发一次）────────────────────────────────
    if face_stable && ~greeted && strcmp(CURRENT_MODE,'idle')
        fprintf('[%s] 新人脸出现 → 打招呼\n', robot_utils.ts());

        % 举手打招呼
        q_greet = robot_utils.buildConfig(R.q_home, R.arm_greet, R.g_5, ...
                                          R.ARM_IDX, R.HAND_IDX);
        traj = robot_utils.smoothTraj(q_current, q_greet, R.TRAJ_N_SLOW);
        q_current = robot_utils.execTraj(traj, R.robot, ax_robot, ...
            '你好！👋', V.cam, hImg, 2);
        pause(0.6);

        % ★ 打招呼完成后回到完全初始位置 q_home ★
        traj = robot_utils.smoothTraj(q_current, R.q_home, R.TRAJ_N_MID);
        q_current = robot_utils.execTraj(traj, R.robot, ax_robot, ...
            '归零 → 等待指令', V.cam, hImg, 2);

        greeted = true;
        fprintf('[%s] 打招呼完成，已归零\n', robot_utils.ts());
    end

    % ── 手势触发模式切换（实物用，同时对电脑也有效）────────────────
    %    识别到"点赞"持续 SWITCH_HOLD_FRAMES 帧 → 切换到下一个模式
    if strcmp(CURRENT_MODE,'idle') && face_stable && greeted && do_dl
        [sw_label, sw_conf] = vision_utils.detectGesture(frame, V);
        is_thumb = strcmp(sw_label,'thumb') || ...
                   (V.use_dl && sw_conf > 0.8 && strcmp(sw_label,'thumb'));

        % 传统CV方案：点赞手势单独检测（拇指伸直其余握拳）
        if ~V.use_dl
            is_thumb = detectThumb(frame);
        end

        if is_thumb
            switch_count = switch_count + 1;
        else
            switch_count = 0;
        end

        % 进度条显示在画面上
        if switch_count > 10
            pct = switch_count / SWITCH_HOLD_FRAMES;
            vision_utils.updateSwitchProgress(hImg, pct);
        end

        if switch_count >= SWITCH_HOLD_FRAMES
            switch_count = 0;
            % 找当前模式在序列中的位置，切换到下一个
            cur_idx  = find(strcmp(MODE_SEQ, CURRENT_MODE));
            next_idx = mod(cur_idx, length(MODE_SEQ)) + 1;
            next_mode = MODE_SEQ{next_idx};
            if ~strcmp(next_mode,'idle')
                setappdata(fig_cam,'mode', next_mode);
                fprintf('[%s] 手势切换 → %s\n', robot_utils.ts(), next_mode);
            end
        end
    else
        if ~strcmp(CURRENT_MODE,'idle')
            switch_count = 0;
        end
    end

    % ── 键盘 / 手势 触发的模式切换执行 ─────────────────────────────
    new_mode = getappdata(fig_cam,'mode');
    if ~strcmp(new_mode,'idle') && ~strcmp(new_mode,CURRENT_MODE) ...
       && ~strcmp(new_mode,'switch')

        CURRENT_MODE = new_mode;
        fprintf('[%s] 进入模式: %s\n', robot_utils.ts(), CURRENT_MODE);

        switch CURRENT_MODE
            case 'demo'
                q_current = mode_demo(R, ax_robot, V.cam, hImg, q_current);
            case 'mirror'
                q_current = mode_mirror(R, V, ax_robot, hImg, q_current, fig_cam);
            case 'rps'
                q_current = mode_rps(R, V, ax_robot, hImg, q_current, fig_cam);
        end

        setappdata(fig_cam,'mode','idle');
        CURRENT_MODE = 'idle';

        % 各模式结束后也回到 q_home
        if max(abs(q_current - R.q_home)) > 0.01
            traj = robot_utils.smoothTraj(q_current, R.q_home, R.TRAJ_N_SLOW);
            q_current = robot_utils.execTraj(traj, R.robot, ax_robot, ...
                '模式结束，归零', V.cam, hImg, 2);
        end
    end

    % ── 摄像头显示（待机） ───────────────────────────────────────────
    if strcmp(CURRENT_MODE,'idle')
        if face_stable && greeted
            pct  = switch_count / SWITCH_HOLD_FRAMES;
            stxt = sprintf('等待指令  [1]演示 [2]镜像 [3]猜拳 | 点赞3秒切换(%.0f%%)', pct*100);
        elseif face_stable
            stxt = '检测到人脸，正在打招呼...';
        else
            stxt = '待机 — 等待人脸出现';
        end
        vision_utils.updateCamView(hImg, frame, bbox, stxt, '');
        drawnow limitrate;
    end
end

% ────────────────────────────────────────────────────────────────────────
%  清理
% ────────────────────────────────────────────────────────────────────────
try
    release(V.pointTracker);
    clear V.cam;
catch; end
fprintf('\n系统已退出。\n');


% ========================================================================
%  键盘回调
% ========================================================================
function onKey(src, evt)
    switch evt.Key
        case 'q',      setappdata(src,'quit',true);
        case '1',      setappdata(src,'mode','demo');   fprintf('[键盘] → 演示\n');
        case '2',      setappdata(src,'mode','mirror'); fprintf('[键盘] → 镜像\n');
        case '3',      setappdata(src,'mode','rps');    fprintf('[键盘] → 猜拳\n');
        case 'escape', setappdata(src,'mode','switch'); fprintf('[键盘] ESC退出当前模式\n');
    end
end

% ========================================================================
%  传统CV方案的点赞检测（拇指伸直+其余握拳）
%  独立于普通手势识别，专门用于模式切换触发
% ========================================================================
function is_thumb = detectThumb(frame)
    is_thumb = false;
    try
        [H,W,~] = size(frame);
        % 取画面中央偏下区域
        roi = frame(round(H*0.3):H, round(W*0.2):round(W*0.8), :);
        roi_s = imresize(roi, 0.75);
        ycbcr = rgb2ycbcr(roi_s);
        Cb = double(ycbcr(:,:,2)); Cr = double(ycbcr(:,:,3));
        skin = (Cb>=77)&(Cb<=127)&(Cr>=133)&(Cr<=173);
        se   = strel('disk',4);
        mask = imclose(imopen(skin,se),se);
        mask = imfill(mask,'holes');
        cc   = bwconncomp(mask);
        if cc.NumObjects==0, return; end
        areas = cellfun(@numel,cc.PixelIdxList);
        [maxA,idx] = max(areas);
        if maxA < 2000, return; end
        hm = false(size(mask)); hm(cc.PixelIdxList{idx})=true;
        % 点赞特征：手的纵横比偏高（拇指向上使轮廓偏窄偏高）
        %           填充率偏高（其余四指握拳，凸缺陷少）
        props = regionprops(hm,'BoundingBox');
        if isempty(props), return; end
        bb = props(1).BoundingBox;
        aspect = bb(4)/bb(3);  % height/width，点赞时 > 1.3
        B  = bwboundaries(hm,'noholes');
        if isempty(B), return; end
        cnt = B{1};
        kh  = convhull(cnt(:,2),cnt(:,1));
        hp  = cnt(kh,:);
        ha  = polyarea(hp(:,2),hp(:,1));
        fr  = maxA/ha;
        % 点赞：高纵横比 + 高填充率（握拳无指缝）
        is_thumb = (aspect > 1.3) && (fr > 0.82);
    catch
        is_thumb = false;
    end
end
