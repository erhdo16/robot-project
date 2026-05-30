% ========================================================================
%  core/vision_utils.m  v3
% ========================================================================
classdef vision_utils
methods(Static)

    % ================================================================
    %  人脸追踪更新
    %  修复：initialize 前无条件 release，防止跨模式重入报错
    % ================================================================
    function [face_stable, V, bbox] = updateFaceTrack(frame, V, do_detect)
        bbox = [];
        gray = rgb2gray(frame);
        DW   = round(V.CAM_W * V.DET_SCALE);
        DH   = round(V.CAM_H * V.DET_SCALE);

        if do_detect
            small    = imresize(gray, [DH DW]);
            bb_small = step(V.faceDetector, small);

            if ~isempty(bb_small)
                [~,ix] = max(bb_small(:,3).*bb_small(:,4));
                bb     = bb_small(ix,:) / V.DET_SCALE;
                bbox   = bb;

                x1 = max(1, round(bb(1)));
                y1 = max(1, round(bb(2)));
                x2 = min(V.CAM_W, round(bb(1)+bb(3)));
                y2 = min(V.CAM_H, round(bb(2)+bb(4)));
                face_roi = gray(y1:y2, x1:x2);

                pts = detectMinEigenFeatures(face_roi, 'MinQuality', 0.01);
                if ~isempty(pts)
                    pts_full = pts.Location + [x1-1, y1-1];
                    % 无论 tracker_init 是什么状态，先 release 再 initialize
                    try, release(V.pointTracker); catch; end
                    initialize(V.pointTracker, pts_full, gray);
                    V.tracker_init = true;
                end
            end
        end

        face_stable = false;
        if V.tracker_init
            try
                [~, validity] = step(V.pointTracker, gray);
                n_valid = sum(validity);
                face_stable = (n_valid >= V.TRACK_MIN_PTS);
                if ~face_stable
                    try, release(V.pointTracker); catch; end
                    V.tracker_init = false;
                end
            catch
                V.tracker_init = false;
            end
        end
    end

    % ================================================================
    %  手势识别入口（face_bbox 可选，传入后屏蔽人脸区域）
    % ================================================================
    function [label, conf] = detectGesture(frame, V, face_bbox)
        if nargin < 3, face_bbox = []; end
        if V.use_dl
            [label, conf] = vision_utils.dlGesture(frame, V, face_bbox);
        else
            [label, conf] = vision_utils.cvGesture(frame, face_bbox);
        end
    end

    % ── 深度学习推理 ───────────────────────────────────────────────
    function [label, conf] = dlGesture(frame, V, face_bbox)
        label=''; conf=0;
        if nargin < 3, face_bbox = []; end
        try
            [H,W,~] = size(frame);
            row_start = round(H * 0.50);
            col_start = round(W * 0.10);
            col_end   = round(W * 0.90);
            roi = frame(row_start:H, col_start:col_end, :);

            if ~isempty(face_bbox)
                fx1 = round(face_bbox(1)) - col_start + 1;
                fy1 = round(face_bbox(2)) - row_start + 1;
                fx2 = fx1 + round(face_bbox(3)) - 1;
                fy2 = fy1 + round(face_bbox(4)) - 1;
                [rH, rW, ~] = size(roi);
                fx1 = max(1,fx1); fy1 = max(1,fy1);
                fx2 = min(rW,fx2); fy2 = min(rH,fy2);
                if fx2 > fx1 && fy2 > fy1
                    roi(fy1:fy2, fx1:fx2, :) = 0;
                end
            end

            roi_in = imresize(roi, V.img_size);
            if size(roi_in,3)==1, roi_in=repmat(roi_in,[1 1 3]); end
            [pred, scores] = classify(V.gesture_net, roi_in);
            label = char(pred);
            conf  = max(scores);
        catch
            label=''; conf=0;
        end
    end

    % ── 传统CV：肤色+凸包 ─────────────────────────────────────────
    function [label, conf] = cvGesture(frame, face_bbox)
        label=''; conf=0;
        if nargin < 2, face_bbox = []; end
        try
            [H,W,~] = size(frame);
            row_start = round(H * 0.50);
            col_start = round(W * 0.10);
            col_end   = round(W * 0.90);
            roi   = frame(row_start:H, col_start:col_end, :);
            roi_s = imresize(roi, 0.75);

            ycbcr = rgb2ycbcr(roi_s);
            Cb = double(ycbcr(:,:,2));
            Cr = double(ycbcr(:,:,3));
            skin = (Cb>=77)&(Cb<=127)&(Cr>=133)&(Cr<=173);
            se   = strel('disk',4);
            mask = imclose(imopen(skin,se),se);
            mask = imfill(mask,'holes');
            cc   = bwconncomp(mask);
            if cc.NumObjects == 0, return; end
            areas = cellfun(@numel, cc.PixelIdxList);

            if ~isempty(face_bbox)
                scale = 0.75;
                roi_col_offset = (col_start-1)*scale;
                roi_row_offset = (row_start-1)*scale;
                fx1_roi = face_bbox(1)*scale - roi_col_offset;
                fy1_roi = face_bbox(2)*scale - roi_row_offset;
                fx2_roi = fx1_roi + face_bbox(3)*scale;
                fy2_roi = fy1_roi + face_bbox(4)*scale;
                face_area_roi = face_bbox(3)*scale * face_bbox(4)*scale;
                keep = true(1, cc.NumObjects);
                [rH_s, rW_s] = size(mask);
                for i = 1:cc.NumObjects
                    [rows, cols] = ind2sub([rH_s rW_s], cc.PixelIdxList{i});
                    bx1=min(cols); by1=min(rows); bx2=max(cols); by2=max(rows);
                    ix1=max(bx1,fx1_roi); iy1=max(by1,fy1_roi);
                    ix2=min(bx2,fx2_roi); iy2=min(by2,fy2_roi);
                    if ix2>ix1 && iy2>iy1
                        inter = (ix2-ix1)*(iy2-iy1);
                        blob_area = (bx2-bx1)*(by2-by1)+1;
                        if inter/min(blob_area, face_area_roi+1) > 0.4
                            keep(i) = false;
                        end
                    end
                end
                valid_idx = find(keep);
                if isempty(valid_idx), return; end
                areas = areas(valid_idx);
                cc.PixelIdxList = cc.PixelIdxList(valid_idx);
                cc.NumObjects   = numel(valid_idx);
            end

            [maxA, idx] = max(areas);
            if maxA < 2500, return; end
            [rH_s, rW_s] = size(mask);
            hm = false(rH_s, rW_s);
            hm(cc.PixelIdxList{idx}) = true;

            B = bwboundaries(hm,'noholes');
            if isempty(B), return; end
            cnt = B{1};
            kh  = convhull(cnt(:,2), cnt(:,1));
            hp  = cnt(kh,:);
            ha  = polyarea(hp(:,2), hp(:,1));
            if ha < 1, return; end
            fr = maxA / ha;

            perimeter   = numel(cnt);
            circularity = 4*pi*maxA / (perimeter^2+1);
            nv = vision_utils.countValleys(cnt, hm);

            if circularity > 0.70 && nv == 0
                props = regionprops(hm,'BoundingBox');
                if ~isempty(props)
                    bb_h = props(1).BoundingBox;
                    if bb_h(4)/(bb_h(3)+1) < 1.4, return; end
                end
            end

            [label, conf_raw] = vision_utils.classify09(nv, fr, hm, circularity);
            if ~isempty(label)
                conf = 0.55 + 0.20*conf_raw;
            end
        catch
            label=''; conf=0;
        end
    end

    function n = countValleys(contour, mask)
        n = 0;
        try
            hm2 = poly2mask(contour(:,2),contour(:,1),size(mask,1),size(mask,2));
            dm  = hm2 & ~mask;
            cc2 = bwconncomp(dm);
            for i = 1:cc2.NumObjects
                if numel(cc2.PixelIdxList{i}) > 150, n = n+1; end
            end
        catch; n=0; end
    end

    function [label, quality] = classify09(nv, fr, hm, circularity)
        label=''; quality=0.5;
        if nargin < 4, circularity=0; end
        if nv==0
            if fr>0.88 && circularity>0.55
                label='0'; quality=0.8;
            elseif fr<0.78
                props=regionprops(hm,'BoundingBox');
                if ~isempty(props)
                    bb=props(1).BoundingBox;
                    if bb(4)/(bb(3)+1)>2.0, label='1'; quality=0.7; end
                end
            end
        elseif nv==1
            if fr>0.80, label='8'; quality=0.65;
            else,        label='2'; quality=0.70; end
        elseif nv==2
            label='3'; quality=0.75;
        elseif nv==3
            if fr>=0.70, label='9'; quality=0.65;
            else,         label='4'; quality=0.70; end
        elseif nv>=4
            if fr<0.72
                label='5'; quality=0.75;
            else
                props=regionprops(hm,'BoundingBox');
                if ~isempty(props)
                    bb=props(1).BoundingBox;
                    if bb(3)/(bb(4)+1)>1.3, label='6'; quality=0.65;
                    else,                    label='7'; quality=0.65; end
                end
            end
        end
    end

    % ================================================================
    %  摄像头画面更新
    % ================================================================
    function updateCamView(hImg, frame, bbox, statusTxt, modeTxt)
        if nargin<5, modeTxt=''; end
        disp_frame = frame;
        if ~isempty(bbox)
            disp_frame = insertShape(disp_frame,'Rectangle',bbox,...
                'Color','green','LineWidth',2);
        end
        set(hImg,'CData',disp_frame);
        ax = ancestor(hImg,'axes');
        vision_utils.setOverlayText(ax,'status_txt',[10 20],statusTxt,'yellow');
        if ~isempty(modeTxt)
            vision_utils.setOverlayText(ax,'mode_txt',[10 45],modeTxt,[1 0.6 0]);
        end
    end

    function setOverlayText(ax, tag, pos, str, color)
        t = findobj(ax,'Type','text','Tag',tag);
        if isempty(t)
            text(ax,pos(1),pos(2),str,'Color',color,'FontSize',11,...
                'FontWeight','bold','BackgroundColor',[0 0 0 0.5],...
                'Tag',tag,'Units','pixels','Interpreter','none');
        else
            t(1).String=str; t(1).Color=color;
        end
    end

    function updateSwitchProgress(hImg, pct)
        ax  = ancestor(hImg,'axes');
        fig = ancestor(ax,'figure');
        pct = min(pct,1);
        p = findobj(fig,'Tag','switch_bar');
        if isempty(p)
            annotation(fig,'rectangle',[0,0,pct,0.03],...
                'FaceColor',[0.2 0.8 0.2],'EdgeColor','none','Tag','switch_bar');
        else
            p(1).Position(3)=pct;
        end
        vision_utils.setOverlayText(ax,'switch_txt',[10 460],...
            sprintf('%.0f%%',pct*100),[0.2 1 0.2]);
    end

end
end
