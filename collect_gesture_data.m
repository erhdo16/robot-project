% ========================================================================
%  collect_gesture_data.m — 手势数据采集工具
%
%  使用方法:
%    运行脚本 → 按屏幕提示 → 对每个数字做手势 → 按空格采集
%    完成后自动整理到 gesture_dataset/ 文件夹
%
%  采集建议:
%    每个数字至少采集 60 张
%    分3轮采集：正面光、侧光、稍暗环境各20张
%    手的距离变化：近/中/远各若干张
% ========================================================================

clear; clc; close all;

SAVE_DIR    = 'gesture_dataset';
IMG_SIZE    = [227 227];       % 直接存成网络输入尺寸
TARGET_NUM  = 60;              % 每类目标采集数量
DIGITS      = 0:9;

% 打开摄像头
try
    cam = webcam(1);
    cam.Resolution = '640x480';
catch
    error('无法打开摄像头');
end

% 创建目录
for d = DIGITS
    folder = fullfile(SAVE_DIR, num2str(d));
    if ~isfolder(folder), mkdir(folder); end
end

% 图形界面
fig = figure('Name','数据采集工具','Position',[100 100 900 600]);
set(fig,'KeyPressFcn',@(s,e) setappdata(s,'key',e.Key));
setappdata(fig,'key','');

ax_cam  = subplot(1,2,1,'Parent',fig);
ax_prev = subplot(1,2,2,'Parent',fig);
hImg    = imshow(zeros(480,640,3,'uint8'),'Parent',ax_cam);
title(ax_prev,'最近采集的图片','FontSize',11);

fprintf('===========================================\n');
fprintf('  手势数据采集工具\n');
fprintf('  空格键 = 采集一张\n');
fprintf('  S键    = 跳到下一个数字\n');
fprintf('  Q键    = 退出\n');
fprintf('===========================================\n\n');

for d = DIGITS
    folder    = fullfile(SAVE_DIR, num2str(d));
    existing  = numel(dir(fullfile(folder,'*.png')));
    collected = 0;
    needed    = max(0, TARGET_NUM - existing);

    if needed == 0
        fprintf('数字 %d: 已有%d张，跳过\n', d, existing);
        continue;
    end

    fprintf('\n>>> 请做数字 [%d] 的手势，需再采集 %d 张\n', d, needed);
    fprintf('    空格采集 / S跳过 / Q退出\n');

    while collected < needed
        % 抓帧
        frame = snapshot(cam);

        % 提取ROI（与推理时保持一致）
        [H,W,~] = size(frame);
        r1=round(H*0.30); r2=H;
        c1=round(W*0.15); c2=round(W*0.85);
        roi = frame(r1:r2, c1:c2, :);

        % 显示摄像头画面 + 进度
        frame_disp = insertShape(frame,'Rectangle',[c1,r1,c2-c1,r2-r1],...
            'Color','cyan','LineWidth',2);
        frame_disp = insertText(frame_disp,[10,10],...
            sprintf('数字[%d]  已采集:%d/%d', d, collected+existing, TARGET_NUM),...
            'FontSize',16,'BoxColor','black','TextColor','yellow');
        set(hImg,'CData',frame_disp);
        title(ax_cam, sprintf('数字 [%d] — 空格采集', d), 'FontSize',13);

        drawnow limitrate;

        % 检查按键
        key = getappdata(fig,'key');
        setappdata(fig,'key','');

        if strcmp(key,'q')
            fprintf('用户退出采集\n');
            clear cam; return;
        elseif strcmp(key,'s')
            fprintf('跳过数字 %d\n', d);
            break;
        elseif strcmp(key,'space')
            % 自动寻找未使用的文件名序号，避免覆盖已有文件
            idx = 1;
            while true
                fname = fullfile(folder, sprintf('%d_%04d.png', d, idx));
                if ~isfile(fname)
                    break; % 找到没有被占用的序号，跳出循环
                end
                idx = idx + 1;
            end

            % 采集：调整尺寸后保存
            img_save = imresize(roi, IMG_SIZE);
            imwrite(img_save, fname);
            collected = collected + 1;

            % 在右侧预览最新采集图
            imshow(img_save,'Parent',ax_prev);
            title(ax_prev, sprintf('已采集 %d 张', existing+collected),'FontSize',11);
            drawnow;
            fprintf('  ✓ 采集第 %d 张 → %s\n', existing+collected, fname);
        end
    end

    fprintf('数字 %d 完成：共 %d 张\n', d, existing+collected);
end

clear cam;
fprintf('\n========================================\n');
fprintf('  采集完毕！数据保存在 %s/\n', SAVE_DIR);
fprintf('  现在可以运行 train_gesture_net.m\n');
fprintf('========================================\n');

% 统计汇总
fprintf('\n各类数量:\n');
for d = DIGITS
    folder = fullfile(SAVE_DIR, num2str(d));
    n = numel(dir(fullfile(folder,'*.png')));
    bar_str = repmat('█', 1, floor(n/3));
    fprintf('  [%d]: %3d张  %s\n', d, n, bar_str);
end
