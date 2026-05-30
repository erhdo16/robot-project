% ========================================================================
%  modes/mode_mirror.m — 功能2：镜像模式
% ========================================================================
function q_current = mode_mirror(R, V, ax_robot, hImg, q_current, fig_cam)

ru = robot_utils;

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
responded     = false;   % 当前手势是否已响应过，防止重复触发

fprintf('[镜像模式] 启动，对摄像头展示 0-9 手势\n');
fprintf('[镜像模式] 请将脸移出画面后再展示手势\n');
fprintf('[镜像模式] 按键盘 1/2/3 切换模式，ESC 停止\n');

while ishandle(fig_cam) && isequal(getappdata(fig_cam,'mode'),'mirror')
    frame_count = frame_count + 1;
    frame = snapshot(V.cam);

    do_detect = (mod(frame_count, V.DETECT_INTERVAL) == 1);
    do_dl     = (mod(frame_count, V.DL_INTERVAL)     == 1);

    [face_visible, V, cached_bbox] = vision_utils.updateFaceTrack(frame, V, do_detect);

    if face_visible
        cached_label = '';
        cached_conf  = 0;
        stable_count = 0;
        last_label   = '';
        responded    = false;
        stxt = '⚠ 检测到人脸，请将脸移出画面后再比手势';
        vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式2: 镜像');
        drawnow limitrate;
        continue;
    end

    if do_dl
        [cached_label, cached_conf] = vision_utils.detectGesture(frame, V, []);
    end

    % 手势变化时重置防抖和响应标志
    if strcmp(cached_label, last_label)
        stable_count = stable_count + 1;
    else
        stable_count = 1;
        last_label   = cached_label;
        responded    = false;   % 新手势，允许再次响应
    end

    conf_ok = (V.use_dl && cached_conf > 0.85) || (~V.use_dl && cached_conf > 0.6);

    % 稳定帧数达标 且 本手势尚未响应过 → 执行一次
    if stable_count >= V.STABLE_THRESH && ~isempty(cached_label) && conf_ok && ~responded
        if GDICT.isKey(cached_label)
            info  = GDICT(cached_label);
            hand  = info{1};
            lname = info{2};
            q_new = ru.buildConfig(R.q_home, R.arm_lo, hand, R.ARM_IDX, R.HAND_IDX);
            traj  = ru.smoothTraj(q_current, q_new, R.TRAJ_N_FAST);
            fprintf('[镜像模式] 识别到 %s (%.0f%%) → 响应\n', lname, cached_conf*100);
            q_current = ru.execTraj(traj, R.robot, ax_robot, ...
                ['镜像: ' lname], V.cam, hImg, 2);
            responded = true;   % 标记已响应，同一手势不再重复
        end
    end

    if ~isempty(cached_label) && stable_count > 0
        if responded
            stxt = sprintf('已响应[%s] — 换个手势继续', cached_label);
        else
            stxt = sprintf('识别[%s] 置信:%.0f%% 稳定:%d/%d', ...
                cached_label, cached_conf*100, ...
                min(stable_count, V.STABLE_THRESH), V.STABLE_THRESH);
        end
    else
        stxt = '镜像模式 | 请展示手势 0-9（脸移出画面）';
    end
    vision_utils.updateCamView(hImg, frame, cached_bbox, stxt, '模式2: 镜像');
    drawnow limitrate;
end

end
