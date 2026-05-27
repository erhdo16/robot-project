%% ============================================================
%  手势舞 主程序 GestureDance_Main.m
%  作者：小组项目
%
%  【动作设计说明】
%  动作1 —— "打招呼" (空间位置1: 手臂侧前方举起)
%    步骤A: 剪刀手"耶" — 食指+中指伸直，其余弯曲
%    步骤B: 布手"挥手" — 五指张开，腕部轻微偏航摆动×3次
%
%  动作2 —— "点赞" (空间位置2: 手臂正前方抬起)
%    步骤A: 大拇指竖起，四指握拳，肘部抬高
%
%  任务覆盖：
%    任务1: 建立运动学模型（importrobot + getTransform正运动学）
%    任务2: 3种可区分手势（剪刀/布/点赞）+ 欠驱动耦合模型
%    任务3: 五次多项式关节空间轨迹规划
%    任务4: 动作1在位置1展示，动作2在位置2展示（≥2个空间位置）
%% ============================================================
clear; clc; close all;

%% ════════════════════════════════════════════════════════════
%  第0节：导入机器人模型
%% ════════════════════════════════════════════════════════════

% 从URDF文件导入机器人刚体树模型
robot = importrobot('JingChuJointR.urdf');

% 设置关节角以行向量格式存储（1×52），便于矩阵运算
robot.DataFormat = 'row';

% 获取home位姿配置（所有关节角=0），作为运动基准
q_home = homeConfiguration(robot);
fprintf('机器人可控关节数: %d\n', length(q_home));

%% ════════════════════════════════════════════════════════════
%  第1节：任务1 —— 运动学模型 & 关节索引建立
%% ════════════════════════════════════════════════════════════
%
%  【关节索引说明】
%  robot.showdetails()显示54个body，但其中2个是fixed关节:
%    body7  (llegsite)  —— 左脚固定连接，不进入配置向量
%    body14 (rlegsite)  —— 右脚固定连接，不进入配置向量
%  因此配置向量长度 = 54 - 2 = 52
%
%  索引映射规则：body编号 → 配置向量下标
%    body 1~6   → 配置 1~6    （左腿6关节）
%    body 7     → 跳过（fixed）
%    body 8~13  → 配置 7~12   （右腿6关节）
%    body 14    → 跳过（fixed）
%    body 15~54 → 配置 13~52  （腰/头/双臂/双手）
%
%  右臂7关节: body 37~43 → 配置索引 35~41
%    35: right_shoulder_pitch  肩部俯仰（前后抬臂）
%    36: right_shoulder_roll   肩部外展（侧向抬臂）
%    37: right_shoulder_yaw    肩部旋转（大臂内外旋）
%    38: right_elbow_pitch     肘部俯仰（屈肘）
%    39: right_elbow_yaw       肘部旋转（前臂旋前/旋后）
%    40: right_wrist_pitch     腕部俯仰
%    41: right_wrist_yaw       腕部偏航
%
%  右手11关节: body 44~54 → 配置索引 42~52
%    42: right_index_finger_2_pitch   食指近节
%    43: right_index_finger_3_pitch   食指远节（耦合关节）
%    44: right_little_finger_2_pitch  小指近节
%    45: right_little_finger_3_pitch  小指远节（耦合关节）
%    46: right_middle_finger_2_pitch  中指近节
%    47: right_middle_finger_3_pitch  中指远节（耦合关节）
%    48: right_ring_finger_2_pitch    无名指近节
%    49: right_ring_finger_3_pitch    无名指远节（耦合关节）
%    50: right_thumb_1_roll           拇指侧展（内收/外展）
%    51: right_thumb_2_pitch          拇指近节
%    52: right_thumb_3_pitch          拇指远节（耦合关节）

% 定义右臂和右手的配置向量索引范围
ARM_IDX  = (37:43) - 2;   % → [35,36,37,38,39,40,41]
HAND_IDX = (44:54) - 2;   % → [42,43,44,45,46,47,48,49,50,51,52]

% 末端执行器body名称（腕关节末端，用于正运动学计算末端位姿）
EE_BODY = 'right_wrist_yaw';

% ── 正运动学验证 ──────────────────────────────────────────
% getTransform 计算从base到EE_BODY的4×4齐次变换矩阵T
% T = [R, p; 0 0 0 1]，其中R是旋转矩阵，p是末端位置(x,y,z)
T_home = getTransform(robot, q_home, EE_BODY);
fprintf('\n=== 任务1: 正运动学验证 ===\n');
fprintf('所有关节归零时，右腕末端坐标:\n');
fprintf('  x = %.3f m （左右方向）\n', T_home(1,4));
fprintf('  y = %.3f m （前后方向）\n', T_home(2,4));
fprintf('  z = %.3f m （上下方向）\n', T_home(3,4));

%% ════════════════════════════════════════════════════════════
%  第2节：任务2 —— 欠驱动手势设计
%% ════════════════════════════════════════════════════════════
%
%  【欠驱动耦合机制说明】
%  右手有5个驱动源（电机），控制11个物理关节
%  每根手指：1个电机驱动近节(2_pitch)，远节(3_pitch)通过
%  机械耦合被动跟随，比例约为 0.7（可根据实物调整）
%
%  耦合公式: q_远节 = COUPLE × q_近节
%
%  大拇指额外有1_roll关节控制侧向展开方向

COUPLE = 0.7;               % 欠驱动耦合比例系数

BEND = deg2rad(80);         % 手指完全弯曲角度
HALF = deg2rad(30);         % 手指半弯（用于过渡）
T_OUT = deg2rad(35);        % 拇指向外侧展（正常握持方向）
T_IN  = deg2rad(-20);       % 拇指向内收（点赞时贴近手背）

% ── 手势构造函数 ──────────────────────────────────────────
% 输入各手指驱动角度，输出11维手部关节配置向量
% 参数: idx=食指, lit=小指, mid=中指, rng=无名指
%       th_r=拇指侧展, th_p=拇指弯曲, c=耦合系数
function h = makeHand(idx, lit, mid, rng, th_r, th_p, c)
    h = zeros(1,11);          % 初始化11个手部关节为0
    h(1) = idx;               % 食指近节
    h(2) = c * idx;           % 食指远节（耦合）
    h(3) = lit;               % 小指近节
    h(4) = c * lit;           % 小指远节（耦合）
    h(5) = mid;               % 中指近节
    h(6) = c * mid;           % 中指远节（耦合）
    h(7) = rng;               % 无名指近节
    h(8) = c * rng;           % 无名指远节（耦合）
    h(9) = th_r;              % 拇指侧展（不耦合，独立关节）
    h(10) = th_p;             % 拇指近节
    h(11) = c * th_p;         % 拇指远节（耦合）
end

% ✌️ 剪刀手"耶"：食指+中指完全伸直，其余弯曲
%    食指=0(伸直), 小指=BEND(弯曲), 中指=0(伸直),
%    无名指=BEND(弯曲), 拇指外展+半弯
g_scissors = makeHand(0,    BEND, 0,    BEND, T_OUT, HALF, COUPLE);

% 🖐 布"挥手"：五指全部伸直张开，拇指自然外展
%    所有弯曲角=0，拇指侧展保持外展方向
g_paper    = makeHand(0,    0,    0,    0,    T_OUT, 0,    COUPLE);

% 👍 点赞：四指握拳弯曲，拇指向上竖起（侧展角度向内收使拇指朝上）
%    食指/小指/中指/无名指全弯，拇指侧展=T_IN，拇指弯曲=0（竖直）
g_thumb    = makeHand(BEND, BEND, BEND, BEND, T_IN,  0,    COUPLE);

fprintf('\n=== 任务2: 手势定义完成 ===\n');
fprintf('  ✌️  剪刀手: 食指+中指伸直，其余弯曲\n');
fprintf('  🖐  布手:   五指全部伸直\n');
fprintf('  👍  点赞:   四指握拳，拇指竖起\n');

%% ════════════════════════════════════════════════════════════
%  第3节：辅助函数定义
%% ════════════════════════════════════════════════════════════

% buildConfig: 将手臂7个角度和手部11个角度写入完整52维配置向量
% 输入: q_base=基础配置, arm7=手臂角度, hand11=手部角度
% 输出: q=完整52维配置向量
function q = buildConfig(q_base, arm7, hand11, aidx, hidx)
    q = q_base;               % 复制基础配置（保留腿部/腰部关节不变）
    q(aidx)  = arm7;          % 写入7个手臂关节角度
    q(hidx)  = hand11;        % 写入11个手部关节角度
end

% smoothTraj: 五次多项式关节空间轨迹插值
% 输入: q_start=起始配置(1×52), q_end=终止配置(1×52), n=插值点数
% 输出: traj=轨迹矩阵(n×52)，每行是一个时刻的完整关节配置
%
% 五次多项式 s(t) = 6t⁵ - 15t⁴ + 10t³，t∈[0,1]
% 该多项式保证: s(0)=0, s(1)=1, s'(0)=s'(1)=0（速度平滑）
%              s''(0)=s''(1)=0（加速度平滑，无冲击）
function traj = smoothTraj(q_start, q_end, n)
    t = linspace(0, 1, n);               % 时间参数，均匀分布 1×n
    s = 6*t.^5 - 15*t.^4 + 10*t.^3;    % 五次多项式插值系数 1×n
    % repmat将q_start复制n行得到 n×52 的基础矩阵
    % s'是 n×1 列向量，(q_end-q_start)是 1×52 行向量
    % 外积 s' * (q_end-q_start) = n×52，表示每个时刻的增量
    traj = repmat(q_start, n, 1) + s' * (q_end - q_start);
end

%% ════════════════════════════════════════════════════════════
%  第4节：任务3 —— 关节空间轨迹规划
%  【空间位置说明】
%  位置1（打招呼位置）: 手臂侧前方举起，类似人类挥手
%    肩俯仰+45°(前屈), 肩外展-50°(侧举), 肘部朝上旋转
%    末端大约在机器人右侧偏前、肩膀高度位置
%
%  位置2（点赞位置）: 手臂正前方抬起，肘部弯曲上扬
%    肩俯仰+35°(前屈), 肩外展-15°(轻微外展), 肘屈曲
%    末端大约在机器人正前方、胸部高度偏上位置
%% ════════════════════════════════════════════════════════════
fprintf('\n=== 任务3: 轨迹规划 ===\n');

% ── 位置1: 打招呼位置（手臂侧前方举起）──────────────────
% 各分量含义: [肩俯仰, 肩外展, 肩旋转, 肘俯仰, 肘旋转, 腕俯仰, 腕偏航]
% 正值方向: 俯仰=前屈, 外展=内收(负=外展), 旋转=内旋
% 【右臂外展方向说明】
% right_shoulder_roll 对右臂而言：
%   正值 = 外展（手臂向右侧抬起，远离身体）✓
%   负值 = 内收（手臂向身体中心靠拢）✗
% 这与左臂符号相反（镜像关系），切勿搞混

arm_wave = deg2rad([ 45,  +50,  -10,   80,   60,   10,    0]);
%                    ^肩前屈 ^右臂外展(正值!)  ^大臂外旋 ^屈肘  ^前臂旋使肘朝上 ^腕

% ── 位置2: 点赞位置（手臂正前偏上，肘部抬高）────────────
arm_thumb = deg2rad([ 35,  +15,  -30,   60,   70,  -10,    0]);
%                    ^肩前屈 ^轻微外展(正值!) ^大臂旋  ^屈肘  ^肘部朝上旋  ^腕轻微

% ── 挥手动作：腕部偏航左右摆动（3次）────────────────────
% 通过改变腕部偏航角(第7个手臂关节)实现挥手效果
WRIST_YAW_IDX = ARM_IDX(7);  % 腕偏航在配置向量中的绝对索引(=41)
WAVE_AMP = deg2rad(20);       % 挥手幅度±20°

% ── 各关键帧配置构建 ──────────────────────────────────────
% 剪刀手在位置1（打招呼起始手势）
q_scissors_wave  = buildConfig(q_home, arm_wave,  g_scissors, ARM_IDX, HAND_IDX);

% 布手在位置1（挥手：腕部3个不同偏航角度，模拟摆动）
q_paper_wave0    = buildConfig(q_home, arm_wave,  g_paper,    ARM_IDX, HAND_IDX);
% 挥手位置A: 腕向左偏+20°
q_paper_waveL    = q_paper_wave0;
q_paper_waveL(WRIST_YAW_IDX) = WAVE_AMP;
% 挥手位置B: 腕向右偏-20°
q_paper_waveR    = q_paper_wave0;
q_paper_waveR(WRIST_YAW_IDX) = -WAVE_AMP;

% 点赞手在位置2
q_thumb_pos      = buildConfig(q_home, arm_thumb, g_thumb,    ARM_IDX, HAND_IDX);

% ── 每段插值帧数设置 ──────────────────────────────────────
N_slow = 80;   % 慢速段（手臂大幅度移动，需要更多帧保证平滑）
N_fast = 35;   % 快速段（挥手摆动，幅度小可以用更少帧）
N_mid  = 60;   % 中速段

% ── 拼接完整演示序列 ──────────────────────────────────────
% 序列: 归零→剪刀→布→挥左→挥右→挥左→回正→点赞→归零
traj_all = [
    smoothTraj(q_home,         q_scissors_wave,  N_slow);  % 举臂做剪刀手
    smoothTraj(q_scissors_wave, q_paper_wave0,   N_mid);   % 剪刀→布（展开手指）
    smoothTraj(q_paper_wave0,   q_paper_waveL,   N_fast);  % 挥手向左
    smoothTraj(q_paper_waveL,   q_paper_waveR,   N_fast);  % 挥手向右
    smoothTraj(q_paper_waveR,   q_paper_waveL,   N_fast);  % 挥手向左（第2次）
    smoothTraj(q_paper_waveL,   q_paper_waveR,   N_fast);  % 挥手向右（第2次）
    smoothTraj(q_paper_waveR,   q_paper_wave0,   N_fast);  % 回正
    smoothTraj(q_paper_wave0,   q_thumb_pos,     N_slow);  % 移动到点赞位置
    smoothTraj(q_thumb_pos,     q_home,          N_slow);  % 归位
];

fprintf('轨迹总帧数: %d（约%.1f秒@30fps）\n', ...
    size(traj_all,1), size(traj_all,1)/30);

% ── 绘制手臂关节角轨迹图 ──────────────────────────────────
figure('Name','手臂7关节轨迹','Position',[50 420 1050 360]);
jnames = {'肩俯仰','肩外展','肩旋转','肘俯仰','肘旋转','腕俯仰','腕偏航'};
total_frames = size(traj_all,1);
for i = 1:7
    subplot(2,4,i);
    plot(1:total_frames, rad2deg(traj_all(:, ARM_IDX(i))), ...
         'LineWidth', 1.8, 'Color', [0.15 0.45 0.85]);
    title(jnames{i}, 'FontSize', 10);
    xlabel('帧'); ylabel('角度(°)'); grid on;
    % 标注各动作段分界线
    xlines = cumsum([N_slow, N_mid, N_fast, N_fast, N_fast, N_fast, N_fast, N_slow]);
    for xl = xlines(1:end-1)
        xline(xl, '--r', 'Alpha', 0.4);
    end
end
sgtitle('任务3 — 手臂7关节轨迹（五次多项式插值）', 'FontSize', 12);

% ── 绘制手部关节角轨迹图 ──────────────────────────────────
figure('Name','手部11关节轨迹','Position',[50 30 1100 290]);
fnames = {'食近','食远','小近','小远','中近','中远','环近','环远','拇展','拇近','拇远'};
for i = 1:11
    subplot(2,6,i);
    plot(1:total_frames, rad2deg(traj_all(:, HAND_IDX(i))), ...
         'LineWidth', 1.5, 'Color', [0.85 0.35 0.15]);
    title(fnames{i}, 'FontSize', 9);
    xlabel('帧'); grid on;
end
sgtitle('任务3 — 手部11关节轨迹（欠驱动耦合）', 'FontSize', 12);

%% ════════════════════════════════════════════════════════════
%  第5节：任务4 —— 多空间位置验证
%% ════════════════════════════════════════════════════════════
fprintf('\n=== 任务4: 多空间位置验证 ===\n');

% 计算两个姿态下右腕末端的世界坐标
T_wave  = getTransform(robot, q_scissors_wave, EE_BODY);  % 打招呼位置
T_thumb = getTransform(robot, q_thumb_pos,     EE_BODY);  % 点赞位置

fprintf('位置1（打招呼，剪刀手）末端坐标:\n');
fprintf('  x=%.3f m, y=%.3f m, z=%.3f m\n', ...
    T_wave(1,4), T_wave(2,4), T_wave(3,4));
fprintf('位置2（点赞位置）末端坐标:\n');
fprintf('  x=%.3f m, y=%.3f m, z=%.3f m\n', ...
    T_thumb(1,4), T_thumb(2,4), T_thumb(3,4));
fprintf('两空间位置之间距离: %.3f m\n', ...
    norm(T_wave(1:3,4) - T_thumb(1:3,4)));

%% ════════════════════════════════════════════════════════════
%  第6节：3D动画展示
%% ════════════════════════════════════════════════════════════
fprintf('\n=== 开始3D动画展示 ===\n');

% 每段的标签（用于动画标题显示）
seg_ends   = cumsum([N_slow, N_mid, N_fast, N_fast, N_fast, N_fast, N_fast, N_slow, N_slow]);
seg_labels = {'举臂→✌剪刀手', '✌→🖐展开手指', ...
              '🖐挥手←', '🖐挥手→', '🖐挥手←', '🖐挥手→', '🖐回正', ...
              '移动→👍点赞', '👍归位'};

figure('Name','手势舞动画','Position',[250 80 820 620]);
ax = gca;

for i = 1:size(traj_all, 1)
    % 判断当前帧属于哪个动作段（用于显示标签）
    seg = find(i <= seg_ends, 1, 'first');
    if isempty(seg), seg = length(seg_labels); end

    % show()渲染机器人当前帧的3D姿态
    % PreservePlot=false: 每帧清除上一帧（动画效果）
    % Frames='off': 不显示坐标轴框架（画面更简洁）
    show(robot, traj_all(i,:), 'Parent', ax, ...
         'PreservePlot', false, 'Frames', 'off');

    title(ax, ['手势舞动画   ' seg_labels{seg}], ...
          'FontSize', 13, 'FontWeight', 'bold');

    % 设置观察视角：方位角135°（从右前方看），仰角15°
    view(ax, 135, 15);

    % 固定坐标轴范围，防止画面抖动
    % x: 左右, y: 前后, z: 上下（单位：米）
    axis(ax, [-0.8, 0.8, -0.6, 0.8, -0.2, 1.8]);

    drawnow;       % 立即刷新图形窗口
    pause(0.025);  % 暂停25ms ≈ 40fps
end

fprintf('\n✅ 手势舞演示完成！\n');
fprintf('  任务1: 正运动学建立并验证 ✓\n');
fprintf('  任务2: 3种手势（剪刀/布/点赞）+ 欠驱动耦合模型 ✓\n');
fprintf('  任务3: 五次多项式平滑轨迹，含挥手摆动 ✓\n');
fprintf('  任务4: 位置1(侧前方挥手) + 位置2(正前方点赞) ✓\n');
