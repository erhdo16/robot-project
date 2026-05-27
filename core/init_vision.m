% ========================================================================
%  core/init_vision.m
%  摄像头、人脸检测器、PointTracker初始化
%  返回视觉配置结构体 V
% ========================================================================
function V = init_vision()

% ── 摄像头 ───────────────────────────────────────────────────────────
try
    cam = webcam(1);
    cam.Resolution = '640x480';
    fprintf('[init_vision] 摄像头已就绪: %s\n', cam.Name);
catch ME
    error('摄像头打开失败: %s\n实机时请将设备号改为机器人摄像头编号。', ME.message);
end

% ── 人脸检测器（降采样用） ───────────────────────────────────────────
faceDetector = vision.CascadeObjectDetector('FrontalFaceCART', ...
    'MinSize',       [40 40], ...
    'MergeThreshold', 4);

% ── 特征点追踪器（同一张脸只招呼一次）───────────────────────────────
% 用法：检测到人脸时提取角点，交给 tracker 持续追踪
% 追踪点全部消失 → 认为人离开 → 重置打招呼标志
pointTracker = vision.PointTracker('MaxBidirectionalError', 2, ...
                                    'NumPyramidLevels', 3);

V.cam           = cam;
V.faceDetector  = faceDetector;
V.pointTracker  = pointTracker;
V.tracker_init  = false;   % tracker是否已初始化

% ── 检测参数 ─────────────────────────────────────────────────────────
V.CAM_W          = 640;
V.CAM_H          = 480;
V.DET_SCALE      = 0.5;     % 人脸检测缩放比
V.DETECT_INTERVAL= 3;       % 每N帧检测一次人脸
V.DL_INTERVAL    = 4;       % 每N帧运行一次手势DL推理
V.FACE_LOST_MAX  = 30;      % 连续N帧追踪点消失 → 人离开
V.STABLE_THRESH  = 6;       % 手势防抖帧数
V.TRACK_MIN_PTS  = 4;       % 追踪点少于此数 → 视为人脸消失

% ── 深度学习模型（如已训练）─────────────────────────────────────────
MODEL_PATH = 'gesture_net.mat';
if isfile(MODEL_PATH)
    fprintf('[init_vision] 加载手势识别模型...\n');
    md = load(MODEL_PATH);
    V.gesture_net  = md.trained_net;
    V.class_names  = md.class_names;
    V.img_size     = md.img_size;
    V.use_dl       = true;
    fprintf('[init_vision] DL模型已加载\n');
else
    fprintf('[init_vision] 未找到 gesture_net.mat，使用传统CV方案\n');
    V.gesture_net = [];
    V.class_names = [];
    V.img_size    = [227 227];
    V.use_dl      = false;
end

end
