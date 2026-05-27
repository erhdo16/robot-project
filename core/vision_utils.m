% ========================================================================
%  core/vision_utils.m
%  人脸追踪、手势识别、摄像头显示等视觉工具
% ========================================================================
classdef vision_utils
methods(Static)

    % ================================================================
    %  人脸追踪更新
    %  每次调用时：
    %    - 检测帧：重新检测人脸，若有则初始化/刷新tracker
    %    - 非检测帧：用tracker预测当前点位置
    %  返回：
    %    face_stable  — 人脸是否稳定存在（tracker点够多）
    %    V            — 更新后的视觉状态结构体
    %    bbox         — 最新人脸框（可能为[]）
    % ================================================================
    function [face_stable, V, bbox] = updateFaceTrack(frame, V, do_detect)
        bbox = [];
        gray = rgb2gray(frame);
        DW   = round(V.CAM_W * V.DET_SCALE);
        DH   = round(V.CAM_H * V.DET_SCALE);

        if do_detect
            % 降采样检测
            small    = imresize(gray, [DH DW]);
            bb_small = step(V.faceDetector, small);

            if ~isempty(bb_small)
                % 取面积最大的人脸（离摄像头最近的人）
                [~,ix] = max(bb_small(:,3).*bb_small(:,4));
                bb     = bb_small(ix,:) / V.DET_SCALE;
                bbox   = bb;

                % 在人脸区域内提取角点，初始化/重置tracker
                x1=max(1,round(bb(1))); y1=max(1,round(bb(2)));
                x2=min(V.CAM_W,round(bb(1)+bb(3)));
                y2=min(V.CAM_H,round(bb(2)+bb(4)));
                face_roi = gray(y1:y2, x1:x2);

                pts = detectMinEigenFeatures(face_roi, 'MinQuality',0.01);
                if ~isempty(pts)
                    % 坐标转回全图
                    pts_full = pts.Location + [x1-1, y1-1];
                    if V.tracker_init
                        release(V.pointTracker);
                    end
                    initialize(V.pointTracker, pts_full, gray);
                    V.tracker_init = true;
                end
            end
        end

        % 不管是否检测帧，都运行tracker
        face_stable = false;
        if V.tracker_init
            try
                [~, validity] = step(V.pointTracker, gray);
                n_valid = sum(validity);
                face_stable = (n_valid >= V.TRACK_MIN_PTS);
                if ~face_stable
                    % 追踪点太少，认为人已离开，重置tracker
                    release(V.pointTracker);
                    V.tracker_init = false;
                end
            catch
                V.tracker_init = false;
            end
        end
    end

    % ================================================================
    %  手势识别（DL优先，无模型时降级为传统CV）
    %  返回 label（字符串 '0'~'9' 或 ''）和 confidence（0~1）
    % ================================================================
    function [label, conf] = detectGesture(frame, V)
        if V.use_dl
            [label, conf] = vision_utils.dlGesture(frame, V);
        else
            [label, conf] = vision_utils.cvGesture(frame);
        end
    end

    % ── 深度学习推理 ───────────────────────────────────────────────
    function [label, conf] = dlGesture(frame, V)
        label=''; conf=0;
        try
            [H,W,~]=size(frame);
            roi = frame(round(H*0.30):H, round(W*0.15):round(W*0.85), :);
            roi_in = imresize(roi, V.img_size);
            if size(roi_in,3)==1, roi_in=repmat(roi_in,[1 1 3]); end
            [pred,scores] = classify(V.gesture_net, roi_in);
            label = char(pred);
            conf  = max(scores);
        catch
            label=''; conf=0;
        end
    end

    % ── 传统CV：肤色+凸包 ─────────────────────────────────────────
    function [label, conf] = cvGesture(frame)
        label=''; conf=0;
        try
            [H,W,~]=size(frame);
            roi   = frame(round(H*0.30):H, round(W*0.15):round(W*0.85), :);
            roi_s = imresize(roi, 0.75);
            ycbcr = rgb2ycbcr(roi_s);
            Cb=double(ycbcr(:,:,2)); Cr=double(ycbcr(:,:,3));
            skin=(Cb>=77)&(Cb<=127)&(Cr>=133)&(Cr<=173);
            se=strel('disk',4);
            mask=imclose(imopen(skin,se),se);
            mask=imfill(mask,'holes');
            cc=bwconncomp(mask);
            if cc.NumObjects==0, return; end
            areas=cellfun(@numel,cc.PixelIdxList);
            [maxA,idx]=max(areas);
            if maxA<2000, return; end
            hm=false(size(mask)); hm(cc.PixelIdxList{idx})=true;
            B=bwboundaries(hm,'noholes');
            if isempty(B), return; end
            cnt=B{1};
            kh=convhull(cnt(:,2),cnt(:,1));
            hp=cnt(kh,:);
            ha=polyarea(hp(:,2),hp(:,1));
            if ha<1, return; end
            fr=maxA/ha;
            nv=vision_utils.countValleys(cnt,hm);
            label=vision_utils.classify09(nv,fr,hm);
            conf=0.6;  % 传统CV给固定置信度
        catch
            label=''; conf=0;
        end
    end

    function n = countValleys(contour,mask)
        n=0;
        try
            hm2=poly2mask(contour(:,2),contour(:,1),size(mask,1),size(mask,2));
            dm=hm2&~mask;
            cc2=bwconncomp(dm);
            for i=1:cc2.NumObjects
                if numel(cc2.PixelIdxList{i})>150, n=n+1; end
            end
        catch; n=0; end
    end

    function label=classify09(nv,fr,hm)
        label='';
        if nv==0
            if fr>0.85, label='0'; else, label='1'; end
        elseif nv==1
            if fr>0.78, label='8'; else, label='2'; end
        elseif nv==2
            label='3';
        elseif nv==3
            if fr>=0.68, label='9'; else, label='4'; end
        elseif nv>=4
            if fr<0.72
                label='5';
            else
                props=regionprops(hm,'BoundingBox');
                if ~isempty(props)
                    bb=props(1).BoundingBox;
                    if bb(3)/bb(4)>1.3, label='6'; else, label='7'; end
                end
            end
        end
    end

    % ================================================================
    %  摄像头画面更新（含人脸框+状态文字叠加）
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
        % 状态文字（复用text对象，避免每帧重建）
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

    % ================================================================
    %  手势切换进度条（点赞保持时显示在画面底部）
    %  pct: 0~1
    % ================================================================
    function updateSwitchProgress(hImg, pct)
        ax  = ancestor(hImg,'axes');
        fig = ancestor(ax,'figure');
        pct = min(pct,1);
        % 复用 annotation rectangle 作进度条
        p = findobj(fig,'Tag','switch_bar');
        if isempty(p)
            annotation(fig,'rectangle',[0, 0, pct, 0.03],...
                'FaceColor',[0.2 0.8 0.2],'EdgeColor','none',...
                'Tag','switch_bar');
        else
            p(1).Position(3) = pct;
        end
        vision_utils.setOverlayText(ax,'switch_txt',[10 460],...
            sprintf('保持👍切换模式 %.0f%%', pct*100),[0.2 1 0.2]);
    end

end
end