% ========================================================================
%  modes/mode_mirror.m — 功能2：镜像模式
%
%  持续识别摄像头中的手势（0-9），机器人做出相同手势
%  按 ESC 或主循环切换键退出
%
%  调用方式：
%    q_current = mode_mirror(R, V, ax_robot, hImg, q_current, fig_cam)
% ========================================================================
function q_current = mode_mirror(R, V, ax_robot, hImg, q_current, fig_cam)

ru = robot_utils;

% 手势字典：标签 → {hand向量, 显示名}
GDICT = containers.Map(...
    {'0','1','2','3','4','5','6','7','8','9'}, ...
    {{R.g_0,'[0]'},{R.g_1,'[1]'},{R.g_2,'[2]'},{R.g_3,'[3]'},{R.g_4,'[4]'}, ...
     {R.g_5,'[5]'},{R.g_6,'[6]'},{R.g_7,'[7]'},{R.g_8,'[8]'},{R.g_9,'[9]'}});

last_label    = '';
stable_count  = 0;
frame_count   = 0;
cached_label  = '';
cached_conf   = 0;
cached_bbox   = [];

fprintf('[镜像模式] 启动，对摄像头展示 0-9 手势\n');
fprintf('[镜像模式] 按键盘 1/2/3 切换模式，ESC 停止\n');

while ishandle(fig_cam) && ~isequal(getappdata(fig_cam,'mode'),'switch')
    frame_count = frame_count + 1;
    frame = snapshot(V.cam);

    do_detect = (mod(frame_count, V.DETECT_INTERVAL) == 1);
    do_dl     = (mod(frame_count, V.DL_INTERVAL)     == 1);

    % 人脸追踪（维持打招呼状态，不重复触发）
    [~, V, cached_bbox] = vision_utils.updateFaceTrack(frame, V, do_detect);

    % 手势识别
    if do_dl
        [cached_label, cached_conf] = vision_utils.detectGesture(frame, V);
    end

    % 防抖
    if strcmp(cached_label, last_label)
        stable_count = stable_count + 1;
    else
        stable_count = 1;
        last_label   = cached_label;
    end

    % 触发响应（DL需置信度>0.75，传统CV固定0.6直接通过）
    conf_ok = (V.use_dl && cached_conf > 0.75) || (~V.use_dl && cached_conf > 0.5);

    if stable_count == V.STABLE_THRESH && ~isempty(cached_label) && conf_ok
        if GDICT.isKey(cached_label)
            info  = GDICT(cached_label);
            hand  = info{1};
            lname = info{2};
            q_new = ru.buildConfig(R.q_home, R.arm_lo, hand, R.ARM_IDX, R.HAND_IDX);
            traj  = ru.smoothTraj(q_current, q_new, R.TRAJ_N_FAST);
            fprintf('[镜像模式] 识别到 %s (%.0f%%) → 响应\n', ...
                lname, cached_conf*100);
            q_current = ru.execTraj(traj, R.robot, ax_robot, ...
                ['镜像: ' lname], V.cam, hImg, 2);
            stable_count = 0;
        end
    end

    % 更新摄像头显示
    if ~isempty(cached_label) && stable_count > 0
        stxt = sprintf('识别[%s] 置信:%.0f%% 稳定:%d/%d', ...
            cached_label, cached_conf*100, ...
            min(stable_count,V.STABLE_THRESH), V.STABLE_THRESH);
    else
        stxt = '镜像模式 | 请展示手势 0-9';
    end
    vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式2: 镜像');
    drawnow limitrate;
end

end
