% ========================================================================
%  core/init_robot.m
%  机器人模型初始化、手势库、手臂姿态、轨迹工具函数
%  被 main.m 调用，返回一个包含所有配置的结构体 R
% ========================================================================
function R = init_robot()

% ── URDF路径：kepler_WeidJoint文件夹在robot_project/下 ──────────────
% mfilename('fullpath') = .../robot_project/core/init_robot.m
% fileparts调用两次：core/ → robot_project/
SCRIPT_DIR = fileparts(fileparts(mfilename('fullpath')));
urdf_path  = fullfile(SCRIPT_DIR, 'kepler_WeidJoint', 'JingChuJointR.urdf');

if ~isfile(urdf_path)
    error(['找不到URDF文件: %s\n' ...
           '请确认 kepler_WeidJoint 文件夹已放入 robot_project/ 目录下。'], ...
           urdf_path);
end

robot = importrobot(urdf_path);
robot.DataFormat = 'row';
q_home = homeConfiguration(robot);

R.robot    = robot;
R.q_home   = q_home;
R.ARM_IDX  = (37:43) - 2;
R.HAND_IDX = (44:54) - 2;

% ── 关节角常量 ────────────────────────────────────────────────────────
COUPLE  = 0.7;
BEND    = deg2rad(80);
HALF    = deg2rad(45);
QUAT    = deg2rad(20);
T_OUT   = deg2rad(35);
T_IN    = deg2rad(-20);
T_PINCH = deg2rad(40);

R.COUPLE=COUPLE; R.BEND=BEND; R.HALF=HALF; R.QUAT=QUAT;
R.T_OUT=T_OUT;   R.T_IN=T_IN; R.T_PINCH=T_PINCH;

% ── 手势库（直接用你的定义）─────────────────────────────────────────
mk = @(a,b,c,d,e,f) makeHand(a,b,c,d,e,f,COUPLE);

R.g_0        = mk(BEND, BEND, BEND, BEND, T_IN,  HALF );
R.g_1        = mk(0,    BEND, BEND, BEND, T_IN,  HALF );
R.g_2        = mk(0,    BEND, 0,    BEND, T_IN,  HALF );
R.g_3        = mk(0,    BEND, 0,    BEND, T_OUT, 0    );
R.g_4        = mk(0,    0,    0,    0,    T_IN,  BEND );
R.g_5        = mk(0,    0,    0,    0,    T_OUT, 0    );
R.g_6        = mk(BEND, 0,    BEND, BEND, T_OUT, 0    );
R.g_7        = mk(0,    0,    BEND, BEND, T_IN,  0    );
R.g_8        = mk(0,    BEND, BEND, BEND, T_OUT, 0    );
R.g_9        = mk(QUAT, BEND, BEND, BEND, T_IN,  HALF );
R.g_scissors = mk(0,    BEND, 0,    BEND, T_IN,  BEND );  % 剪刀
R.g_paper    = mk(0,    0,    0,    0,    0,     0    );  % 布
R.g_rock     = mk(BEND, BEND, BEND, BEND, T_IN,  HALF );  % 石头
R.g_thumb    = mk(BEND, BEND, BEND, BEND, 0,     0    );  % 点赞

% ── 手臂姿态 ─────────────────────────────────────────────────────────
% arm_hi : 高位展示（打招呼、0-4演示）
% arm_lo : 低位展示（5-9演示、镜像、猜拳）
R.arm_hi    = deg2rad([ 65, +65, -10,  60,  30,  30,  0]);
R.arm_lo    = deg2rad([ 30, +30, -10,  60, -20,   0, 10]);
R.arm_greet = deg2rad([ 55, +55, -10,  70,  25,   5,  0]); % 打招呼专用

% ── 参数 ─────────────────────────────────────────────────────────────
R.TRAJ_N_SLOW = 80;   % 帧数：慢速过渡（手臂大幅运动）
R.TRAJ_N_MID  = 50;   % 帧数：中速（手势切换）
R.TRAJ_N_FAST = 30;   % 帧数：快速（同位置手势变换）
R.HOLD_SECS   = 1.0;  % 每个演示手势停留秒数

fprintf('[init_robot] 机器人加载完成，关节数=%d\n', length(q_home));
end

% ────────────────────────────────────────────────────────────────────────
function h = makeHand(idx,lit,mid,rng,th_r,th_p,c)
    h=zeros(1,11);
    h(1)=idx; h(2)=c*idx; h(3)=lit; h(4)=c*lit;
    h(5)=mid; h(6)=c*mid; h(7)=rng; h(8)=c*rng;
    h(9)=th_r; h(10)=th_p; h(11)=c*th_p;
end
