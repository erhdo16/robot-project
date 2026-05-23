% ========================================================================
%  gesture_debug.m — 手势识别调试工具
%
%  用途：在接入机器人之前，单独验证手势识别的中间结果
%  包含：
%    模式A — 实时摄像头调试（看各中间步骤）
%    模式B — 静态图片批量测试（可提供测试图片）
%
%  使用方法：
%    直接运行 → 默认进入摄像头实时调试
%    提供图片 → 修改 TEST_IMAGES 变量后运行模式B
% ========================================================================

clear; clc; close all;

% ────────────────────────────────────────────────────────────────────────
%  选择运行模式
%  'camera' — 实时摄像头，按 Q 退出
%  'images' — 静态图片批量测试
% ────────────────────────────────────────────────────────────────────────
MODE = 'camera';

% 模式B：在此填入你的测试图片路径（支持jpg/png）
% 建议每个数字准备2-3张，在不同光线/背景下拍摄
TEST_IMAGES = {
    'hand_0.jpg',   % 数字0的手势图片
    'hand_1.jpg',
    'hand_2.jpg',
    'hand_3.jpg',
    'hand_4.jpg',
    'hand_5.jpg',
};
GROUND_TRUTH = {'0','1','2','3','4','5'};  % 对应正确答案

% ────────────────────────────────────────────────────────────────────────
%  肤色分割参数（这里是最需要调的地方）
% ────────────────────────────────────────────────────────────────────────
% YCbCr 标准肤色范围，根据你的摄像头和光线环境调整
CB_MIN = 77;   CB_MAX = 127;   % 蓝色差分量
CR_MIN = 133;  CR_MAX = 173;   % 红色差分量
% 提示：光线偏暖(黄光)时 CR_MIN 可降低到 125
%       肤色偏深时 CB_MAX 可提高到 135

MORPH_RADIUS  = 4;    % 形态学操作半径，越大越平滑但越慢
MIN_HAND_AREA = 2000; % 最小手部面积(像素²)，太小=噪声，太大=漏检
MIN_VALLEY_SZ = 150;  % 最小凸缺陷面积，过滤指缝噪声

% ────────────────────────────────────────────────────────────────────────
%  调试窗口布局（6格：原图/ROI/肤色/形态学/凸包/结果）
% ────────────────────────────────────────────────────────────────────────
fig = figure('Name','手势识别调试', 'Position',[30 50 1280 700]);
titles = {'原始帧','ROI区域','肤色分割','形态学清洗','凸包+谷点','识别结果'};
ax = gobjects(1,6);
for i = 1:6
    ax(i) = subplot(2,3,i,'Parent',fig);
    title(ax(i), titles{i},'FontSize',10);
end

% ════════════════════════════════════════════════════════════════════════
%  模式A：实时摄像头调试
% ════════════════════════════════════════════════════════════════════════
if strcmp(MODE,'camera')

    try
        cam = webcam(1);
        cam.Resolution = '640x480';
    catch
        error('无法打开摄像头');
    end

    set(fig,'KeyPressFcn',@(s,e) setappdata(s,'quit',strcmp(e.Key,'q')));
    setappdata(fig,'quit',false);

    fprintf('实时调试中，按 Q 退出\n');
    fprintf('观察各窗格，根据提示调整顶部参数\n\n');

    while ishandle(fig) && ~getappdata(fig,'quit')
        frame = snapshot(cam);
        debugVisualize(frame, ax, CB_MIN,CB_MAX,CR_MIN,CR_MAX, ...
                       MORPH_RADIUS, MIN_HAND_AREA, MIN_VALLEY_SZ);
        drawnow limitrate;
    end
    clear cam;

% ════════════════════════════════════════════════════════════════════════
%  模式B：静态图片批量测试
% ════════════════════════════════════════════════════════════════════════
else
    correct = 0;
    total   = length(TEST_IMAGES);
    results = cell(total, 3);  % {文件名, 预测, 真值}

    for i = 1:total
        if ~isfile(TEST_IMAGES{i})
            fprintf('⚠ 找不到文件: %s\n', TEST_IMAGES{i});
            continue;
        end
        img = imread(TEST_IMAGES{i});
        [label, dbg] = debugVisualize(img, ax, CB_MIN,CB_MAX,CR_MIN,CR_MAX, ...
                                       MORPH_RADIUS, MIN_HAND_AREA, MIN_VALLEY_SZ);
        pause(0.8);   % 停留看结果

        gt   = GROUND_TRUTH{i};
        hit  = strcmp(label, gt);
        if hit, correct = correct+1; end

        results{i,1} = TEST_IMAGES{i};
        results{i,2} = label;
        results{i,3} = gt;

        fprintf('图片 %-20s | 预测: %-3s | 真值: %-3s | %s\n', ...
            TEST_IMAGES{i}, label, gt, ternary(hit,'✓','✗'));
    end

    fprintf('\n========================================\n');
    fprintf('准确率: %d / %d = %.1f%%\n', correct, total, 100*correct/total);
    fprintf('========================================\n');

    % 打印失败案例供分析
    fprintf('\n失败案例分析:\n');
    for i = 1:total
        if ~isempty(results{i,1}) && ~strcmp(results{i,2}, results{i,3})
            fprintf('  %s → 预测[%s] 应为[%s]，建议检查肤色参数或光线\n', ...
                results{i,1}, results{i,2}, results{i,3});
        end
    end
end

% ════════════════════════════════════════════════════════════════════════
%  核心调试可视化函数
%  输入: frame — 原始RGB帧
%        ax    — 6个subplot句柄
%  输出: label — 识别结果字符串
% ════════════════════════════════════════════════════════════════════════
function [label, dbg] = debugVisualize(frame, ax, ...
    CB_MIN,CB_MAX,CR_MIN,CR_MAX,MORPH_R,MIN_AREA,MIN_VALLEY)

    label = '';
    dbg   = struct();
    [H,W,~] = size(frame);

    % ── 步骤1: 原图 ──────────────────────────────────────────────────
    imshow(frame,'Parent',ax(1));
    title(ax(1),'①原始帧','FontSize',10);

    % ── 步骤2: ROI提取 ──────────────────────────────────────────────
    r1=round(H*0.30); r2=H;
    c1=round(W*0.15); c2=round(W*0.85);
    roi = frame(r1:r2, c1:c2, :);
    roi_s = imresize(roi, 0.75);

    % 在原图上画ROI框
    frame_roi = insertShape(frame,'Rectangle',[c1,r1,c2-c1,r2-r1],...
        'Color','cyan','LineWidth',2);
    imshow(frame_roi,'Parent',ax(2));
    title(ax(2),sprintf('②ROI [%d×%d]',size(roi_s,2),size(roi_s,1)),'FontSize',10);

    % ── 步骤3: 肤色分割 ─────────────────────────────────────────────
    ycbcr = rgb2ycbcr(roi_s);
    Cb = double(ycbcr(:,:,2));
    Cr = double(ycbcr(:,:,3));
    skin = (Cb>=CB_MIN)&(Cb<=CB_MAX)&(Cr>=CR_MIN)&(Cr<=CR_MAX);

    % 叠加显示：肤色区域涂绿
    roi_skin = roi_s;
    roi_skin(:,:,1) = uint8(double(roi_s(:,:,1)) .* ~skin);
    roi_skin(:,:,2) = uint8(double(roi_s(:,:,2)) .* ~skin + 200*skin);
    roi_skin(:,:,3) = uint8(double(roi_s(:,:,3)) .* ~skin);
    imshow(roi_skin,'Parent',ax(3));
    skin_pct = 100*sum(skin(:))/numel(skin);
    title(ax(3),sprintf('③肤色分割 (%.1f%%像素)',skin_pct),'FontSize',10);
    % 提示: 肤色占比 < 2% → 检测不到手；> 40% → 参数过宽
    if skin_pct < 2
        title(ax(3),sprintf('③肤色分割 %.1f%% ← 太少！调小CB/CR范围',skin_pct),...
            'FontSize',10,'Color','red');
    elseif skin_pct > 40
        title(ax(3),sprintf('③肤色分割 %.1f%% ← 太多！调大CB/CR范围',skin_pct),...
            'FontSize',10,'Color',[1 0.5 0]);
    end

    % ── 步骤4: 形态学清洗 ───────────────────────────────────────────
    se   = strel('disk',MORPH_R);
    mask = imclose(imopen(skin,se),se);
    mask = imfill(mask,'holes');

    cc   = bwconncomp(mask);
    areas = cellfun(@numel, cc.PixelIdxList);

    % 画出所有连通域（用颜色区分大小）
    mask_label = labelmatrix(cc);
    mask_rgb   = label2rgb(mask_label,'jet','k','shuffle');
    imshow(mask_rgb,'Parent',ax(4));

    if isempty(areas)
        title(ax(4),'④形态学 — 无连通域！','FontSize',10,'Color','red');
        imshow(zeros(H,W,3,'uint8'),'Parent',ax(5));
        imshow(zeros(H,W,3,'uint8'),'Parent',ax(6));
        return;
    end

    [maxA,idx] = max(areas);
    title(ax(4),sprintf('④形态学 %d个域，最大=%d px²',cc.NumObjects,maxA),...
        'FontSize',10, 'Color', ternary(maxA>=MIN_AREA,'black','red'));

    if maxA < MIN_AREA
        title(ax(4),sprintf('④最大域%d < 阈值%d ← 调小MIN_HAND_AREA或靠近摄像头',...
            maxA,MIN_AREA),'FontSize',10,'Color','red');
        return;
    end

    hand_mask = false(size(mask));
    hand_mask(cc.PixelIdxList{idx}) = true;

    % ── 步骤5: 凸包 + 谷点可视化 ────────────────────────────────────
    B = bwboundaries(hand_mask,'noholes');
    if isempty(B), return; end
    contour = B{1};

    k_hull   = convhull(contour(:,2),contour(:,1));
    hull_pts = contour(k_hull,:);

    hull_mask = poly2mask(contour(:,2),contour(:,1),size(mask,1),size(mask,2));
    diff_mask = hull_mask & ~hand_mask;
    cc2 = bwconncomp(diff_mask);

    % 统计谷点
    n_valleys = 0;
    valley_centroids = [];
    for i = 1:cc2.NumObjects
        if numel(cc2.PixelIdxList{i}) > MIN_VALLEY
            n_valleys = n_valleys + 1;
            [vr,vc] = ind2sub(size(diff_mask),cc2.PixelIdxList{i});
            valley_centroids(end+1,:) = [mean(vc), mean(vr)]; %#ok<AGROW>
        end
    end

    hull_area  = polyarea(hull_pts(:,2),hull_pts(:,1));
    fill_ratio = maxA / hull_area;

    % 绘制: 手轮廓(白)+凸包(青)+谷点(红圈)
    vis = repmat(uint8(hand_mask)*180, [1 1 3]);
    imshow(vis,'Parent',ax(5)); hold(ax(5),'on');
    plot(ax(5), contour(:,2), contour(:,1), 'w-',  'LineWidth',1);
    plot(ax(5), hull_pts(:,2), hull_pts(:,1),'c--','LineWidth',2);
    if ~isempty(valley_centroids)
        plot(ax(5), valley_centroids(:,1), valley_centroids(:,2), ...
            'ro','MarkerSize',10,'LineWidth',2,'MarkerFaceColor','r');
    end
    hold(ax(5),'off');
    title(ax(5),sprintf('⑤谷点数=%d  填充率=%.2f',n_valleys,fill_ratio),'FontSize',10);

    % ── 步骤6: 分类结果 ──────────────────────────────────────────────
    label = classify09(n_valleys, fill_ratio, hand_mask);

    result_img = zeros(200,300,3,'uint8');
    if ~isempty(label)
        result_str = sprintf('识别结果: %s', label);
        col = [0 230 0];
    else
        result_str = '未识别';
        col = [230 0 0];
    end
    result_img = insertText(result_img,[30,70],result_str,...
        'FontSize',36,'TextColor',col,'BoxColor','black');
    detail_str = sprintf('谷点:%d  填充:%.2f', n_valleys, fill_ratio);
    result_img = insertText(result_img,[30,140],detail_str,...
        'FontSize',18,'TextColor','white','BoxColor','black');
    imshow(result_img,'Parent',ax(6));
    title(ax(6),'⑥识别结果','FontSize',10);

    dbg.n_valleys  = n_valleys;
    dbg.fill_ratio = fill_ratio;
    dbg.skin_pct   = skin_pct;
    dbg.max_area   = maxA;
end

% ── 手势分类（与主程序保持一致）────────────────────────────────────────
function label = classify09(n_valleys, fill_ratio, hand_mask)
    label = '';
    if n_valleys == 0
        label = '1'; if fill_ratio > 0.85, label = '0'; end
    elseif n_valleys == 1
        label = '2'; if fill_ratio > 0.78, label = '8'; end
    elseif n_valleys == 2
        label = '3';
    elseif n_valleys == 3
        label = '4'; if fill_ratio >= 0.68, label = '9'; end
    elseif n_valleys >= 4
        label = '5';
        if fill_ratio >= 0.72
            props = regionprops(hand_mask,'BoundingBox');
            if ~isempty(props)
                bb = props(1).BoundingBox;
                label = '6'; if bb(3)/bb(4) <= 1.3, label = '7'; end
            end
        end
    end
end

% ── 三元运算符 ──────────────────────────────────────────────────────────
function r = ternary(cond, a, b)
    if cond, r=a; else, r=b; end
end
