%% 虚拟对比实验 虚拟麦轮小车可执行轨迹版本：
%% 说明：
%% 1) 本版本不再使用 smoothPathOmni() 三次样条平滑。
%% 2) RRT* 输出路径会被转换为麦轮小车真实可执行轨迹。
%% 3) 输出轨迹 q 中，相邻两个点之间只允许：
%%    前进、后退、左移、右移、左前、右前、左后、右后 8 种方向。
%% 4) 暂不接入真实小车蓝牙控制，仅在 MATLAB / Unity 仿真中验证轨迹坐标形式。

clear;

% ===== 初始化 =====
name = "Matlab";
Client = TCPInit('127.0.0.1', 55016, name);
load('Omni_Calib_Results_Unity.mat');
ocam_model = calib_data.ocam_model;

cvsyst_rot = 0;
camY = 0; camX = 0; camZ = 0;
lasY = 0.0; lasX = 0.0;
las_dist = 1850;

% ===== 起点 / 终点 =====
start_point = [0, 0];
goal_point  = [18000, 50];

robot_trace     = [];
safety_distance = 3000;

% ===== 实时性参数 =====
VIS_DECIMATE_GLOBAL   = 5;
OBSTACLE_EXTRACT_INTERVAL_SEC = 0.5;  % 动态障碍提取真实时间间隔
COLLISION_CHECK_INTERVAL_SEC  = 1;  % 剩余路径碰撞检测真实时间间隔

% ===== 栅格地图参数 =====
GRID_RESOLUTION = 150;
GRID_ORIGIN     = [-20000, -25000];
GRID_SIZE       = [500, 300];

HIT_THRESHOLD  = 1;
DECAY_FRAMES   = 20;  % 约 8~10 秒未观测到的动态障碍逐步清除
DECAY_INTERVAL = 3;

% ===== 局部 RRT* 参数 =====
LOCAL_PLAN_RADIUS    = 8000;
REPLAN_COOLDOWN_SEC  = 2.0;  % 避免连续重规划，同时保持动态响应
SAFETY_DIST_FALLBACK = [safety_distance, safety_distance*0.7, safety_distance*0.5];


% ===== 电机控制仿真参数：模糊自适应 PID =====
CONTROL_DT_NOMINAL = 0.10;  % 首帧及异常计时情况下使用的控制周期 s
CONTROL_DT_MIN = 0.03;          % 防止过小 dt 放大微分噪声
CONTROL_DT_MAX = 0.65;  % 防止重规划/暂停后单帧位置跳变过大
CONTROL_TIME_SCALE = 1.0;  % Unity 仿真时间倍率，1.0 表示与真实时间一致
MAX_CHASSIS_SPEED = 1050;  % 适度降速，提高动态环境制动裕量和 PID 稳定性
ARRIVE_THRESHOLD = 250;  % 到达轨迹点阈值 mm
SLOWDOWN_DISTANCE = 1600;       % 从该距离开始连续减速，避免阶跃式速度切换
MIN_SPEED_RATIO = 0.30;  % 接近轨迹点时降低速度，提高转弯和停车稳定性
MAX_CHASSIS_ACCEL = 1300;  % 降低加速度，减少 PID 瞬态误差
MAX_CHASSIS_REF_STEP = 380;  % 低 FPS 时限制单帧方向切换幅度 mm/s

MAX_WHEEL_SPEED = 1700;  % 与第三版电机模型的实际速度上限保持一致
MAX_PWM = 100;  % PWM 最大值，模拟 Arduino 中 0~100 速度指令
pwm_last = zeros(4,1);
PWM_RATE_LIMIT_PER_SEC = 180;   % PWM 最大变化率，避免性能/FPS 改变控制效果
PWM_RATE_LIMIT_PER_STEP = 32;  % 低 FPS 时避免单帧 PWM 直接大幅反向
chassis_ref_cmd = [0, 0];       % 经过加速度限制后的车体速度指令

% TT 减速直流电机简化一阶模型参数
MOTOR_TAU = 0.18;               % 更接近快速闭环速度响应
MOTOR_GAIN = MAX_WHEEL_SPEED / MAX_PWM;  % PWM 到轮速的比例，mm/s per PWM

% 四个电机当前实际线速度，单位 mm/s
motor_speed_actual = zeros(4,1);

% ===== Experiment 2 controller selection and parallel comparison states =====
% CONTROL_MODE only decides which controller drives the Unity robot.
% All three methods are simulated and recorded in parallel using the same
% wheel-speed reference generated from the same planned path.
% Available modes: "FEEDFORWARD_ONLY", "FIXED_PID", "FUZZY_PID".
CONTROL_MODE = "FUZZY_PID";
CONTROLLER_IDS = ["FEEDFORWARD_ONLY", "FIXED_PID", "FUZZY_PID"];
CONTROLLER_NAMES = ["Feedforward only", "Fixed PID", "Fuzzy adaptive PID"];
active_controller_idx = find(CONTROL_MODE == CONTROLLER_IDS, 1);
if isempty(active_controller_idx)
    error("Unsupported CONTROL_MODE. Use FEEDFORWARD_ONLY, FIXED_PID, or FUZZY_PID.");
end
num_controllers = numel(CONTROLLER_IDS);

% Fixed PID parameters. Units are PWM per wheel-speed error terms.
FIXED_KP = 0.035;
FIXED_KI = 0.0030;
FIXED_KD = 0.0040;
FIXED_PID_CORRECTION_LIMIT = 28;
FIXED_INTEGRAL_LIMIT = 2500;
cmp_fixed_pid_integral = zeros(4,1);
cmp_fixed_pid_prev_error = zeros(4,1);
cmp_fuzzy_pid_state = initFuzzyPIDState3(4);

% Parallel motor/PWM states for the three controllers.
cmp_motor_speed_actual = zeros(4, num_controllers);
cmp_pwm_last = zeros(4, num_controllers);

% ===== Non-ideal motor model for Experiment 2 =====
% The controller still uses nominal MOTOR_GAIN for feedforward, while the
% simulated plant uses wheel-dependent gain errors and random speed noise.
% All controllers receive the same gain error and disturbance sequence.
rng(20260708, "twister");
MOTOR_GAIN_ERROR_SCALE = [0.88; 1.08; 0.93; 1.12];
MOTOR_GAIN_ACTUAL = MOTOR_GAIN * MOTOR_GAIN_ERROR_SCALE;
MOTOR_DISTURBANCE_STD = 35;       % mm/s, Gaussian wheel-speed disturbance
MOTOR_DISTURBANCE_LIMIT = 90;     % mm/s, clamp occasional noise spikes


% ===== PID 控制结果显示参数 =====
PID_PLOT_MAX_POINTS = inf;  % 只保留最近 PID_PLOT_MAX_POINTS 个点，避免数据越来越大
pid_time_log = [];
wheel_ref_log = [];
wheel_actual_log = [];
pwm_log = [];
pwm_ff_log = [];
pwm_pid_log = [];
chassis_speed_log = [];

% ===== 麦轮可执行轨迹参数 =====
% q 中相邻点最大运动距离，单位 mm。
% 值越小，轨迹点越密，越接近规划路径，但计算/显示点更多。
% 建议范围：200~500 mm。
EXEC_STEP_LEN = 2000;

% 为了防止 RRT* 原始路径转换为 8方向折线路径后贴近障碍物，
% 这里可以给规划安全距离增加额外余量。
EXEC_SAFETY_MARGIN = 200;

% ===== 图像读取容错参数 =====
MAX_BAD_FRAMES = 10;

% ===== 初始化栅格地图 =====
grid_hit_count = zeros(GRID_SIZE(1), GRID_SIZE(2), 'int16');
grid_last_seen = zeros(GRID_SIZE(1), GRID_SIZE(2), 'int32');

% ===== 第一帧 =====
black_image = ImageReadTCP_One(Client, 'Center');
binaryImage = las_segm1(black_image);
last_valid_image = black_image;
bad_frame_count = 0;

% ===== 创建图像句柄 =====
figure;
f1 = subplot(1,4,1); h1 = imshow(black_image); title('Original Image');
f2 = subplot(1,4,2); h2 = imshow(binaryImage); title('Laser Extraction');

f3 = subplot(1,4,3);
h3 = scatter([], [], 5, 'filled'); hold on;
xlim([-10000, 15000]); ylim([-10000, 15000]);
title('Ranging Results'); xlabel('X/mm'); ylabel('Y/mm'); grid on;

f4 = subplot(1,4,4);
h4 = scatter([], [], 5, 'filled'); hold on;
xlim([-10000, 15000]); ylim([-10000, 15000]);
title('World Ranging Results'); xlabel('X/mm'); ylabel('Y/mm'); grid on;

figure;
h_globalmap = scatter([], [], 2, 'blue', 'filled'); hold on;
h_trace     = plot(nan, nan, 'g-', 'LineWidth', 1.5);
h_path      = plot(nan, nan, 'm--', 'LineWidth', 1.0);
title('Global Map with Mecanum-Executable Path');
xlabel('X/mm'); ylabel('Y/mm'); grid on;
xlim([-10000, 20000]); ylim([-10000, 20000]);
legend({'Confirmed Obstacles', 'Robot Path', 'Executable Path'}, 'Location', 'best');


% --- Figure 3：当前帧动态观测地图（橙色，实时障碍物 + 轨迹 + 规划路径）---
figure;
h_dyn_map  = scatter([], [], 4, [1.0 0.4 0.0], 'filled'); hold on;
h_trace_d  = plot(nan, nan, 'g-',  'LineWidth', 1.5);
h_path_d   = plot(nan, nan, 'm--', 'LineWidth', 1.0);

title('Dynamic Observation Map (Current Laser Obstacles)');
xlabel('X/mm'); ylabel('Y/mm'); grid on;
xlim([-10000, 20000]); ylim([-10000, 20000]);
legend({'Current frame obstacles','Robot path','Planned path'}, 'Location', 'best');

% --- Figure 4：PID 电机控制结果显示 ---
figure;
set(gcf, 'Name', 'Fuzzy Adaptive PID Motor Control Results');

subplot(2,2,1);
h_wheel_ref_1 = plot(nan, nan, 'r--', 'LineWidth', 1.0); hold on;
h_wheel_act_1 = plot(nan, nan, 'r-',  'LineWidth', 1.5);
h_wheel_ref_2 = plot(nan, nan, 'g--', 'LineWidth', 1.0);
h_wheel_act_2 = plot(nan, nan, 'g-',  'LineWidth', 1.5);
h_wheel_ref_3 = plot(nan, nan, 'b--', 'LineWidth', 1.0);
h_wheel_act_3 = plot(nan, nan, 'b-',  'LineWidth', 1.5);
h_wheel_ref_4 = plot(nan, nan, 'k--', 'LineWidth', 1.0);
h_wheel_act_4 = plot(nan, nan, 'k-',  'LineWidth', 1.5);
grid on;
xlabel('Time / s');
ylabel('Wheel speed / mm/s');
title('Target vs Actual Wheel Speed');
legend({'Ref M1','Act M1','Ref M2','Act M2','Ref M3','Act M3','Ref M4','Act M4'}, ...
       'Location','best');

subplot(2,2,2);
h_pwm_1 = plot(nan, nan, 'r-', 'LineWidth', 1.2); hold on;
h_pwm_2 = plot(nan, nan, 'g-', 'LineWidth', 1.2);
h_pwm_3 = plot(nan, nan, 'b-', 'LineWidth', 1.2);
h_pwm_4 = plot(nan, nan, 'k-', 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('PWM');
title('PWM Output');
legend({'PWM M1','PWM M2','PWM M3','PWM M4'}, 'Location','best');

subplot(2,2,3);
h_vx_ref  = plot(nan, nan, 'r--', 'LineWidth', 1.0); hold on;
h_vx_real = plot(nan, nan, 'r-',  'LineWidth', 1.5);
h_vy_ref  = plot(nan, nan, 'b--', 'LineWidth', 1.0);
h_vy_real = plot(nan, nan, 'b-',  'LineWidth', 1.5);
grid on;
xlabel('Time / s');
ylabel('Chassis speed / mm/s');
title('Chassis Velocity Tracking');
legend({'vx ref','vx real','vy ref','vy real'}, 'Location','best');

subplot(2,2,4);
h_speed_error_1 = plot(nan, nan, 'r-', 'LineWidth', 1.2); hold on;
h_speed_error_2 = plot(nan, nan, 'g-', 'LineWidth', 1.2);
h_speed_error_3 = plot(nan, nan, 'b-', 'LineWidth', 1.2);
h_speed_error_4 = plot(nan, nan, 'k-', 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('Speed error / mm/s');
title('Wheel Speed Tracking Error');
legend({'e M1','e M2','e M3','e M4'}, 'Location','best');

% --- Figure 5: separate feedforward and PID feedback contributions ---
figure;
set(gcf, 'Name', 'PWM Feedforward and PID Correction');

subplot(2,1,1);
h_effort_ff = plot(nan, nan, 'k--', 'LineWidth', 1.2); hold on;
h_effort_pid = plot(nan, nan, 'b-', 'LineWidth', 1.4);
h_effort_total = plot(nan, nan, 'r-', 'LineWidth', 1.4);
grid on;
xlabel('Time / s');
ylabel('Mean absolute PWM');
title('Control Effort Decomposition');
legend({'Feedforward','PID correction','Total command'}, 'Location','best');

subplot(2,1,2);
h_pid_corr_1 = plot(nan, nan, 'r-', 'LineWidth', 1.2); hold on;
h_pid_corr_2 = plot(nan, nan, 'g-', 'LineWidth', 1.2);
h_pid_corr_3 = plot(nan, nan, 'b-', 'LineWidth', 1.2);
h_pid_corr_4 = plot(nan, nan, 'k-', 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('PWM correction');
title('Fuzzy PID Feedback Contribution');
legend({'PID M1','PID M2','PID M3','PID M4'}, 'Location','best');


% ===== 初始路径规划 =====
b = 1;
cvsyst_x = start_point(1);
cvsyst_y = start_point(2);

[x1_init, y1_init] = mapping(binaryImage, cvsyst_rot, 0, 0, ...
                              camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
if size(x1_init, 2) > 1
    x1_init(:,1) = [];
    y1_init(:,1) = [];
end

x2_init = x1_init + cvsyst_x;
y2_init = y1_init + cvsyst_y;
xy2_init = [x2_init(:), y2_init(:)];

[grid_hit_count, grid_last_seen] = updateGridMap(...
    grid_hit_count, grid_last_seen, xy2_init, ...
    GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, 1);

init_obstacles = safeObstacles(xy2_init);

fprintf('[INIT] 起点=(%.0f,%.0f) 终点=(%.0f,%.0f)\n', ...
        start_point(1), start_point(2), goal_point(1), goal_point(2));
fprintf('[INIT] 初始障碍物点数=%d，规划安全距离=%.0fmm，执行余量=%.0fmm\n', ...
        size(init_obstacles,1), safety_distance, EXEC_SAFETY_MARGIN);

if ~isempty(init_obstacles)
    fprintf('[INIT] 障碍物 x 范围: [%.0f, %.0f]\n', ...
            min(init_obstacles(:,1)), max(init_obstacles(:,1)));
    fprintf('[INIT] 障碍物 y 范围: [%.0f, %.0f]\n', ...
            min(init_obstacles(:,2)), max(init_obstacles(:,2)));
end

[path, q] = planMecanumExecutablePath(...
    start_point, goal_point, init_obstacles, ...
    safety_distance + EXEC_SAFETY_MARGIN, EXEC_STEP_LEN);

checkMecanumExecutablePath(q);

% 为了兼容原程序可视化和碰撞检测变量名，smoothed_path 保存可执行轨迹。
smoothed_path = q;

% ===== 可视化初始规划 =====
figure;
if ~isempty(init_obstacles)
    scatter(init_obstacles(:,1), init_obstacles(:,2), 10, 'red', 'filled'); hold on;
else
    hold on;
end
plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_point(1),  goal_point(2),  'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(path(:,1), path(:,2), 'c--', 'LineWidth', 1.2);
plot(q(:,1), q(:,2), 'k-', 'LineWidth', 2);
axis equal; xlabel('X/mm'); ylabel('Y/mm');
title('Initial Path: Raw RRT* vs Mecanum Executable Path');
legend({'Obstacles','Start','Goal','Raw RRT* Path','Mecanum Executable Path'}, ...
       'Location', 'best');
hold off;

cvsyst_x = q(b,1);
cvsyst_y = q(b,2);
current_position = q(b,:);
cmp_position = repmat(current_position, num_controllers, 1);
cmp_time_log = [];
cmp_wheel_ref_log = [];
cmp_wheel_actual_log = [];
cmp_pwm_log = [];
cmp_chassis_ref_log = [];
cmp_chassis_real_log = [];
cmp_position_log = [];
reference_path_log = [];  % Target path points actually used by the controller.

b = 2;   % q(1,:) 是当前位置，从 q(2,:) 开始作为第一个目标点
confirmed_obstacles = zeros(0, 2);

% ===== Experiment 1 metrics: closed-loop dynamic obstacle avoidance =====
% These variables are only used for evaluation and command-window reporting.
GOAL_SUCCESS_THRESHOLD = 300;       % mm
COLLISION_DISTANCE_THRESHOLD = safety_distance;
experiment_success = false;
replan_count = 0;
replan_success_count = 0;
replan_fail_count = 0;
path_conflict_count = 0;
first_path_conflict_time = NaN;
first_successful_replan_time = NaN;
first_replan_response_time = NaN;
replan_time_log_ms = [];
min_obstacle_distance = inf;
safety_violation_count = 0;
path_length_actual = 0;
initial_path_length = computePathLength(q);
last_metric_position = current_position;
goal_error_log = [];
min_obstacle_distance_log = [];
metric_time_log = [];

% ===== 本版本使用同步取帧，避免异步 TCP 图像流错位导致 PNG Read Error =====
fprintf('[INFO] 使用同步取帧模式，不启用 parfeval 异步预取\n');

frame_count = 0;
control_time_elapsed = 0;
last_obstacle_extract_time = -inf;
last_collision_check_time = -inf;
replan_cooldown_until = 0;

t_fetch = 0; t_laser = 0; t_map = 0;
t_grid  = 0; t_coll  = 0; t_ctrl = 0; t_vis = 0;
tic_loop = tic;
control_clock = tic;

% ===== 主循环 =====
while true
    try
        frame_count = frame_count + 1;

        % 使用真实循环时间，使 Unity 中的移动速度不再随 FPS 下降。
        if frame_count == 1
            control_dt = CONTROL_DT_NOMINAL;
        else
            control_dt = toc(control_clock) * CONTROL_TIME_SCALE;
            control_dt = max(min(control_dt, CONTROL_DT_MAX), CONTROL_DT_MIN);
        end
        control_clock = tic;
        control_time_elapsed = control_time_elapsed + control_dt;

        % --- Step 1: 稳健同步取帧 ---
        t0 = tic;
        try
            img_new = ImageReadTCP_One(Client, 'Center');

            if isValidImageFrame(img_new)
                black_image = img_new;
                last_valid_image = img_new;
                bad_frame_count = 0;
            else
                bad_frame_count = bad_frame_count + 1;
                warning('[FETCH] 无效图像帧，使用上一帧。bad_frame_count=%d', bad_frame_count);
                black_image = last_valid_image;
            end
        catch ME_fetch
            bad_frame_count = bad_frame_count + 1;
            warning('[FETCH] 图像读取失败: %s，使用上一帧。bad_frame_count=%d', ...
            char(ME_fetch.message), bad_frame_count);
            black_image = last_valid_image;

            if bad_frame_count >= MAX_BAD_FRAMES
                warning('[FETCH] 连续坏帧过多，暂停 0.5 秒等待图像流恢复');
                pause(0.5);
                bad_frame_count = 0;
            end
        end
        t_fetch = t_fetch + toc(t0)*1000;

        % --- Step 2: 激光提取 ---
        t0 = tic;
        if ~isValidImageFrame(black_image)
            warning('[LASER] Invalid black_image, skip current frame.');
            continue;
        end

        try
            binaryImage = las_segm1(black_image);
        catch ME_laser
            warning('[LASER] Laser extraction failed: %s, skip current frame.', char(ME_laser.message));
            continue;
        end
        t_laser = t_laser + toc(t0)*1000;

        % --- Step 3: Mapping ---
        t0 = tic;
        [x1, y1] = mapping(binaryImage, cvsyst_rot, 0, 0, ...
                           camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
        if size(x1, 2) > 1
            x1(:,1) = [];
            y1(:,1) = [];
        end
        x2 = x1 + cvsyst_x;
        y2 = y1 + cvsyst_y;
        xy2 = [x2(:), y2(:)];
        t_map = t_map + toc(t0)*1000;

        % --- Step 4: 栅格地图更新 ---
        t0 = tic;
        [grid_hit_count, grid_last_seen] = updateGridMap(...
            grid_hit_count, grid_last_seen, xy2, ...
            GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, frame_count);

        if mod(frame_count, DECAY_INTERVAL) == 0
            grid_hit_count = decayGridMap(...
                grid_hit_count, grid_last_seen, frame_count, DECAY_FRAMES);
        end

        if control_time_elapsed - last_obstacle_extract_time >= ...
                OBSTACLE_EXTRACT_INTERVAL_SEC
            confirmed_obstacles = extractConfirmedObstacles(...
                grid_hit_count, GRID_ORIGIN, GRID_RESOLUTION, HIT_THRESHOLD);
            last_obstacle_extract_time = control_time_elapsed;
        end
        t_grid = t_grid + toc(t0)*1000;

        % --- Step 5: 碰撞检测 + 局部重规划 ---
        t0 = tic;
        need_replan = false;
        if control_time_elapsed >= replan_cooldown_until && ...
           ~isempty(confirmed_obstacles) && ...
           control_time_elapsed - last_collision_check_time >= ...
               COLLISION_CHECK_INTERVAL_SEC

            % 只检查当前位置之后的剩余路径，避免车后方障碍触发无效重规划。
            if b <= size(q, 1)
                remaining_path = [current_position; q(b:end,:)];
                need_replan = check_path_collision3(...
                    remaining_path, confirmed_obstacles, safety_distance);
            end
            last_collision_check_time = control_time_elapsed;
        end

        if need_replan
            path_conflict_count = path_conflict_count + 1;
            replan_count = replan_count + 1;
            if isnan(first_path_conflict_time)
                first_path_conflict_time = control_time_elapsed;
            end
            fprintf('[EXP1] Path conflict detected at t=%.2f s, replan #%d starts.\n', ...
                control_time_elapsed, replan_count);
            disp('麦轮可执行轨迹与确认障碍物冲突，启动局部重规划...');

            replan_clock = tic;
            sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
            local_obstacles = filterLocalObstacles(...
                confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
            local_obstacles = safeObstacles(local_obstacles);

            [new_path, new_q] = tryPlanMecanumPath(...
                current_position, sub_goal, local_obstacles, ...
                SAFETY_DIST_FALLBACK + EXEC_SAFETY_MARGIN, EXEC_STEP_LEN);
            replan_elapsed_ms = toc(replan_clock) * 1000;
            replan_time_log_ms = [replan_time_log_ms; replan_elapsed_ms];

            if ~isempty(new_q)
                replan_success_count = replan_success_count + 1;
                if isnan(first_successful_replan_time)
                    first_successful_replan_time = control_time_elapsed;
                    first_replan_response_time = replan_elapsed_ms / 1000;
                end
                fprintf('[EXP1] Replan success: time=%.1f ms, new executable points=%d.\n', ...
                    replan_elapsed_ms, size(new_q,1));
                path = new_path;
                q = new_q;
                smoothed_path = q;
                checkMecanumExecutablePath(q);
                b = 2;
                replan_cooldown_until = control_time_elapsed + REPLAN_COOLDOWN_SEC;
            else
                replan_fail_count = replan_fail_count + 1;
                fprintf('[EXP1] Replan failed: time=%.1f ms.\n', replan_elapsed_ms);
                warning('[REPLAN] 所有安全距离均失败，保持当前可执行轨迹继续执行');
                replan_cooldown_until = control_time_elapsed + REPLAN_COOLDOWN_SEC;
            end
        end
        t_coll = t_coll + toc(t0)*1000;

       % --- Step 6: 基于模糊自适应 PID 的电机速度闭环控制 ---
t0 = tic;

if b <= size(q, 1)

    % 当前目标轨迹点
    target_position = q(b,:);
    reference_path_log = [reference_path_log; target_position];

    % 当前位置到目标点的误差
    error_vec = target_position - current_position;
    % ===== 小误差方向锁定：避免接近水平/垂直路径段产生横纵抖动 =====
    DIR_LOCK_RATIO = 0.15;
    
    if abs(error_vec(2)) < DIR_LOCK_RATIO * abs(error_vec(1))
        error_vec(2) = 0;     % 主要是水平运动，忽略小的 y 误差
    elseif abs(error_vec(1)) < DIR_LOCK_RATIO * abs(error_vec(2))
        error_vec(1) = 0;     % 主要是竖直运动，忽略小的 x 误差
    end
    dist_to_target = norm(error_vec);

    % ===== 到达当前路径点：只切换 b，不在本帧继续执行 PID =====
    % 低 FPS 时单帧位移可能跨过目标点。除了距离阈值，也检测是否已经
    % 越过垂直于当前路径段的目标平面，避免反向追逐刚刚错过的路径点。
    passed_target = false;
    if b > 1
        segment_vec = target_position - q(b-1,:);
        passed_target = dot(current_position - target_position, segment_vec) >= 0;
    end

    if dist_to_target < ARRIVE_THRESHOLD || passed_target
        fprintf('[ARRIVE] 到达当前路径点 b=%d/%d dist=%.1f\n', ...
            b, size(q,1), dist_to_target);

        b = b + 1;

        % 如果当前路径结束，判断是否需要规划下一段
        if b > size(q,1)
            dist_to_goal = norm(current_position - goal_point);

            if dist_to_goal > 500
                disp('接近子目标，规划下一段麦轮可执行轨迹...');

                sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
                local_obstacles = filterLocalObstacles(...
                    confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
                local_obstacles = safeObstacles(local_obstacles);

                [new_path, new_q] = tryPlanMecanumPath(...
                    current_position, sub_goal, local_obstacles, ...
                    SAFETY_DIST_FALLBACK + EXEC_SAFETY_MARGIN, EXEC_STEP_LEN);

                if ~isempty(new_q)
                    path = new_path;
                    q = new_q;

                    % 强制保证新路径起点等于当前真实位置
                    q(1,:) = current_position;

                    smoothed_path = q;

                    if isempty(q) || size(q,1) < 2
                        error('[PATH] q is empty or has insufficient points.');
                    end

                    fprintf('[PATH] 新可执行轨迹点数 = %d\n', size(q,1));
                    checkMecanumExecutablePath(q);

                    b = 2;
                    replan_cooldown_until = control_time_elapsed + REPLAN_COOLDOWN_SEC;
                end
            else
                fprintf('[NAV] 已到达最终目标点。\n');
                experiment_success = true;
                pwm_last = zeros(4,1);
                motor_speed_actual = zeros(4,1);
                chassis_ref_cmd = [0, 0];
                func_Car(Client, current_position, 1);
                break;
            end
        end

        t_ctrl = t_ctrl + toc(t0)*1000;
        continue;
    end

    % ===== 1. 由目标坐标计算车体目标速度 =====
    direction_vec = error_vec / dist_to_target;

    % 连续速度规划：远处高速巡航，接近轨迹点时平滑减速。
    % sqrt 曲线比线性减速更积极，不会在中距离过早降速。
    distance_ratio = (dist_to_target - ARRIVE_THRESHOLD) / ...
        (SLOWDOWN_DISTANCE - ARRIVE_THRESHOLD);
    speed_ratio = sqrt(max(min(distance_ratio, 1), 0));
    speed_ratio = max(speed_ratio, MIN_SPEED_RATIO);
    target_speed = MAX_CHASSIS_SPEED * speed_ratio;

    chassis_ref_target = direction_vec * target_speed;

    % 对车体目标速度做二维加速度限制。轨迹方向突然改变时先平滑减速，
    % 再沿新方向加速，降低轮速误差尖峰与 PWM 饱和。
    ref_delta = chassis_ref_target - chassis_ref_cmd;
    max_ref_delta = min(MAX_CHASSIS_ACCEL * control_dt, MAX_CHASSIS_REF_STEP);
    if norm(ref_delta) > max_ref_delta
        ref_delta = ref_delta / norm(ref_delta) * max_ref_delta;
    end
    chassis_ref_cmd = chassis_ref_cmd + ref_delta;

    vx_ref = chassis_ref_cmd(1);   % X+：右移
    vy_ref = chassis_ref_cmd(2);   % Y+：前进
    wz_ref = 0;

    % ===== 2. 麦轮运动学：车体速度 -> 四轮目标速度 =====
    motor_speed_ref = mecanumInverseKinematics(vx_ref, vy_ref, wz_ref);
    fprintf('[REF] dx=%.1f dy=%.1f vx_ref=%.1f vy_ref=%.1f wheel_ref=[%.1f %.1f %.1f %.1f]\n', ...
    error_vec(1), error_vec(2), vx_ref, vy_ref, ...
    motor_speed_ref(1), motor_speed_ref(2), motor_speed_ref(3), motor_speed_ref(4));

    motor_speed_ref = max(min(motor_speed_ref, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED);

    % ===== 3. Parallel motor control for Experiment 2 =====
    % All controllers use the same reference wheel speed so that the
    % comparison is based on the same planned path and same velocity command.
    pwm_ff = motor_speed_ref / MOTOR_GAIN;
    pwm_ff_all = repmat(pwm_ff, 1, num_controllers);
    pwm_pid_all = zeros(4, num_controllers);
    pwm_cmd_all = zeros(4, num_controllers);

    % Method 1: feedforward only.
    pwm_cmd_all(:,1) = pwm_ff;

    % Method 2: feedforward + fixed-parameter PID.
    fixed_error = motor_speed_ref - cmp_motor_speed_actual(:,2);
    cmp_fixed_pid_integral = cmp_fixed_pid_integral + fixed_error * control_dt;
    cmp_fixed_pid_integral = max(min(cmp_fixed_pid_integral, FIXED_INTEGRAL_LIMIT), ...
                                 -FIXED_INTEGRAL_LIMIT);
    fixed_derivative = (fixed_error - cmp_fixed_pid_prev_error) / max(control_dt, 1e-3);
    pwm_pid_all(:,2) = FIXED_KP * fixed_error + ...
                       FIXED_KI * cmp_fixed_pid_integral + ...
                       FIXED_KD * fixed_derivative;
    cmp_fixed_pid_prev_error = fixed_error;
    pwm_pid_all(:,2) = max(min(pwm_pid_all(:,2), FIXED_PID_CORRECTION_LIMIT), ...
                           -FIXED_PID_CORRECTION_LIMIT);
    pwm_cmd_all(:,2) = pwm_ff + pwm_pid_all(:,2);

    % Method 3: feedforward + fuzzy adaptive PID.
    for pid_i = 1:4
        [pwm_pid_all(pid_i,3), cmp_fuzzy_pid_state(pid_i)] = fuzzyAdaptivePIDCorrection3(...
            motor_speed_ref(pid_i), cmp_motor_speed_actual(pid_i,3), ...
            cmp_fuzzy_pid_state(pid_i), control_dt);
    end
    PID_CORRECTION_LIMIT = 28;
    pwm_pid_all(:,3) = max(min(pwm_pid_all(:,3), PID_CORRECTION_LIMIT), ...
                           -PID_CORRECTION_LIMIT);
    pwm_cmd_all(:,3) = pwm_ff + pwm_pid_all(:,3);

    % Apply common PWM saturation, rate limit, non-ideal motor model and
    % forward kinematics. Disturbance is shared across controllers to keep
    % the comparison fair.
    cmp_chassis_real_current = zeros(num_controllers, 3);
    shared_disturbance = MOTOR_DISTURBANCE_STD * randn(4,1);
    shared_disturbance = max(min(shared_disturbance, MOTOR_DISTURBANCE_LIMIT), ...
                             -MOTOR_DISTURBANCE_LIMIT);
    for ctrl_i = 1:num_controllers
        pwm_cmd_all(:,ctrl_i) = max(min(pwm_cmd_all(:,ctrl_i), MAX_PWM), -MAX_PWM);
        delta_pwm = pwm_cmd_all(:,ctrl_i) - cmp_pwm_last(:,ctrl_i);
        pwm_rate_limit = min(PWM_RATE_LIMIT_PER_SEC * control_dt, ...
            PWM_RATE_LIMIT_PER_STEP);
        delta_pwm = max(min(delta_pwm, pwm_rate_limit), -pwm_rate_limit);
        pwm_cmd_all(:,ctrl_i) = cmp_pwm_last(:,ctrl_i) + delta_pwm;
        cmp_pwm_last(:,ctrl_i) = pwm_cmd_all(:,ctrl_i);

        cmp_motor_speed_actual(:,ctrl_i) = simulateTTMotorWithGainError(...
            cmp_motor_speed_actual(:,ctrl_i), pwm_cmd_all(:,ctrl_i), ...
            control_dt, MOTOR_TAU, MOTOR_GAIN_ACTUAL, MAX_WHEEL_SPEED);
        cmp_motor_speed_actual(:,ctrl_i) = cmp_motor_speed_actual(:,ctrl_i) + shared_disturbance;
        cmp_motor_speed_actual(:,ctrl_i) = max(min(cmp_motor_speed_actual(:,ctrl_i), ...
            MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED);

        [vx_tmp, vy_tmp, wz_tmp] = mecanumForwardKinematics(cmp_motor_speed_actual(:,ctrl_i));
        cmp_chassis_real_current(ctrl_i,:) = [vx_tmp, vy_tmp, wz_tmp];
    end

    % Select the controller that actually drives the robot in Unity.
    pwm_cmd = pwm_cmd_all(:,active_controller_idx);
    pwm_ff = pwm_ff_all(:,active_controller_idx);
    pwm_pid = pwm_pid_all(:,active_controller_idx);
    motor_speed_actual = cmp_motor_speed_actual(:,active_controller_idx);
    vx_real = cmp_chassis_real_current(active_controller_idx,1);
    vy_real = cmp_chassis_real_current(active_controller_idx,2);
    wz_real = cmp_chassis_real_current(active_controller_idx,3);

    % ===== 记录 PID 控制结果，用于实时显示 =====
    pid_time_log = [pid_time_log; control_time_elapsed];
    
    wheel_ref_log = [wheel_ref_log; motor_speed_ref(:)'];
    wheel_actual_log = [wheel_actual_log; motor_speed_actual(:)'];
    pwm_log = [pwm_log; pwm_cmd(:)'];
    pwm_ff_log = [pwm_ff_log; pwm_ff(:)'];
    pwm_pid_log = [pwm_pid_log; pwm_pid(:)'];
    
    chassis_speed_log = [chassis_speed_log; ...
        vx_ref, vy_ref, vx_real, vy_real];

    % Record all three controllers for Experiment 2 comparison.
    cmp_time_log = [cmp_time_log; control_time_elapsed];
    cmp_wheel_ref_log = [cmp_wheel_ref_log; motor_speed_ref(:)'];
    cmp_wheel_actual_log = [cmp_wheel_actual_log; ...
        cmp_motor_speed_actual(:,1)', cmp_motor_speed_actual(:,2)', cmp_motor_speed_actual(:,3)'];
    cmp_pwm_log = [cmp_pwm_log; ...
        pwm_cmd_all(:,1)', pwm_cmd_all(:,2)', pwm_cmd_all(:,3)'];
    cmp_chassis_ref_log = [cmp_chassis_ref_log; vx_ref, vy_ref];
    cmp_chassis_real_log = [cmp_chassis_real_log; ...
        cmp_chassis_real_current(1,1:2), cmp_chassis_real_current(2,1:2), cmp_chassis_real_current(3,1:2)];
    
    % 只保留最近 PID_PLOT_MAX_POINTS 个点，避免数据越来越大
    if length(pid_time_log) > PID_PLOT_MAX_POINTS
        pid_time_log = pid_time_log(end-PID_PLOT_MAX_POINTS+1:end);
        wheel_ref_log = wheel_ref_log(end-PID_PLOT_MAX_POINTS+1:end, :);
        wheel_actual_log = wheel_actual_log(end-PID_PLOT_MAX_POINTS+1:end, :);
        pwm_log = pwm_log(end-PID_PLOT_MAX_POINTS+1:end, :);
        pwm_ff_log = pwm_ff_log(end-PID_PLOT_MAX_POINTS+1:end, :);
        pwm_pid_log = pwm_pid_log(end-PID_PLOT_MAX_POINTS+1:end, :);
        chassis_speed_log = chassis_speed_log(end-PID_PLOT_MAX_POINTS+1:end, :);
    end

    fprintf('[ACT] pwm=[%.1f %.1f %.1f %.1f] wheel=[%.1f %.1f %.1f %.1f] vx=%.1f vy=%.1f\n', ...
    pwm_cmd(1), pwm_cmd(2), pwm_cmd(3), pwm_cmd(4), ...
    motor_speed_actual(1), motor_speed_actual(2), motor_speed_actual(3), motor_speed_actual(4), ...
    vx_real, vy_real);

    % ===== 6. 根据实际速度积分更新小车坐标 =====
    cmp_position = cmp_position + cmp_chassis_real_current(:,1:2) * control_dt;
    cmp_position_log = [cmp_position_log; ...
        cmp_position(1,:), cmp_position(2,:), cmp_position(3,:)];
    current_position = cmp_position(active_controller_idx,:);

    cvsyst_x = current_position(1);
    cvsyst_y = current_position(2);

    robot_trace = [robot_trace; current_position];

    % ===== Experiment 1 metric update =====
    step_distance = norm(current_position - last_metric_position);
    path_length_actual = path_length_actual + step_distance;
    last_metric_position = current_position;

    current_goal_error = norm(current_position - goal_point);
    current_min_obstacle_distance = inf;
    if ~isempty(confirmed_obstacles)
        current_min_obstacle_distance = min(vecnorm(confirmed_obstacles - current_position, 2, 2));
        min_obstacle_distance = min(min_obstacle_distance, current_min_obstacle_distance);
        if current_min_obstacle_distance < COLLISION_DISTANCE_THRESHOLD
            safety_violation_count = safety_violation_count + 1;
        end
    end
    metric_time_log = [metric_time_log; control_time_elapsed];
    goal_error_log = [goal_error_log; current_goal_error];
    min_obstacle_distance_log = [min_obstacle_distance_log; current_min_obstacle_distance];

    % ===== 7. 发送实际位置到 Unity =====
    q_send = current_position;
    func_Car(Client, q_send, 1);

    fprintf('[MOTOR PID] dt=%.3f b=%d/%d target=(%.0f,%.0f) pos=(%.0f,%.0f) dist=%.1f vx=%.1f vy=%.1f\n', ...
        control_dt, ...
        b, size(q,1), target_position(1), target_position(2), ...
        current_position(1), current_position(2), dist_to_target, vx_real, vy_real);

else
    dist_to_goal = norm(current_position - goal_point);

    if dist_to_goal > 300
        fprintf('[NAV] 当前可执行轨迹结束，距终点 %.0f mm，规划下一段...\n', dist_to_goal);

        sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
        local_obstacles = filterLocalObstacles(...
            confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
        local_obstacles = safeObstacles(local_obstacles);

        [new_path, new_q] = tryPlanMecanumPath(...
            current_position, sub_goal, local_obstacles, ...
            SAFETY_DIST_FALLBACK + EXEC_SAFETY_MARGIN, EXEC_STEP_LEN);

        if ~isempty(new_q)
            path = new_path;
            q = new_q;

            % 强制新路径从当前真实位置开始
            q(1,:) = current_position;

            smoothed_path = q;

            if isempty(q) || size(q,1) < 2
                error('[PATH] q is empty or has insufficient points.');
            end

            fprintf('[PATH] 新可执行轨迹点数 = %d\n', size(q,1));
            checkMecanumExecutablePath(q);

            b = 2;
            replan_cooldown_until = control_time_elapsed + REPLAN_COOLDOWN_SEC;
        end
    else
        fprintf('[NAV] 已到达最终目标点。\n');
        experiment_success = true;
        pwm_last = zeros(4,1);
        motor_speed_actual = zeros(4,1);
        chassis_ref_cmd = [0, 0];
        func_Car(Client, current_position, 1);
        break;
    end
end

t_ctrl = t_ctrl + toc(t0)*1000;

        % --- Step 7: 可视化 ---
       if mod(frame_count, 3) == 0 && ~isempty(pid_time_log)
        t0 = tic;
        set(h1, 'CData', black_image);
        set(h2, 'CData', binaryImage);
        set(h3, 'XData', x1, 'YData', y1);
        % Figure 3：当前帧动态障碍物实时刷新
        if ~isempty(xy2)
            set(h_dyn_map, 'XData', xy2(:,1), 'YData', xy2(:,2));
        else
            set(h_dyn_map, 'XData', [], 'YData', []);
        end
        
        set(h_trace_d, 'XData', robot_trace(:,1), 'YData', robot_trace(:,2));
        set(h_path_d,  'XData', smoothed_path(:,1), 'YData', smoothed_path(:,2));
        if mod(frame_count, VIS_DECIMATE_GLOBAL) == 0
            set(h4, 'XData', x2, 'YData', y2);
            set(h_trace, 'XData', robot_trace(:,1), 'YData', robot_trace(:,2));
            set(h_path,  'XData', smoothed_path(:,1), 'YData', smoothed_path(:,2));

            if ~isempty(confirmed_obstacles)
                set(h_globalmap, 'XData', confirmed_obstacles(:,1), ...
                                 'YData', confirmed_obstacles(:,2));
            else
                set(h_globalmap, 'XData', [], 'YData', []);
            end
        end
        

        % ===== Figure 4：PID 控制结果实时显示 =====
        if ~isempty(pid_time_log)
        
            % 1) 四轮目标速度与实际速度
            if isvalid(h_wheel_ref_1)
                set(h_wheel_ref_1, 'XData', pid_time_log, 'YData', wheel_ref_log(:,1));
                set(h_wheel_act_1, 'XData', pid_time_log, 'YData', wheel_actual_log(:,1));
        
                set(h_wheel_ref_2, 'XData', pid_time_log, 'YData', wheel_ref_log(:,2));
                set(h_wheel_act_2, 'XData', pid_time_log, 'YData', wheel_actual_log(:,2));
        
                set(h_wheel_ref_3, 'XData', pid_time_log, 'YData', wheel_ref_log(:,3));
                set(h_wheel_act_3, 'XData', pid_time_log, 'YData', wheel_actual_log(:,3));
        
                set(h_wheel_ref_4, 'XData', pid_time_log, 'YData', wheel_ref_log(:,4));
                set(h_wheel_act_4, 'XData', pid_time_log, 'YData', wheel_actual_log(:,4));
            end
        
            % 2) PWM 输出
            if isvalid(h_pwm_1)
                set(h_pwm_1, 'XData', pid_time_log, 'YData', pwm_log(:,1));
                set(h_pwm_2, 'XData', pid_time_log, 'YData', pwm_log(:,2));
                set(h_pwm_3, 'XData', pid_time_log, 'YData', pwm_log(:,3));
                set(h_pwm_4, 'XData', pid_time_log, 'YData', pwm_log(:,4));
            end

            % Figure 5: expose the part generated by PID instead of only
            % showing the total PWM, which naturally resembles wheel speed.
            if isvalid(h_effort_ff)
                set(h_effort_ff, 'XData', pid_time_log, ...
                    'YData', mean(abs(pwm_ff_log), 2));
                set(h_effort_pid, 'XData', pid_time_log, ...
                    'YData', mean(abs(pwm_pid_log), 2));
                set(h_effort_total, 'XData', pid_time_log, ...
                    'YData', mean(abs(pwm_log), 2));

                set(h_pid_corr_1, 'XData', pid_time_log, 'YData', pwm_pid_log(:,1));
                set(h_pid_corr_2, 'XData', pid_time_log, 'YData', pwm_pid_log(:,2));
                set(h_pid_corr_3, 'XData', pid_time_log, 'YData', pwm_pid_log(:,3));
                set(h_pid_corr_4, 'XData', pid_time_log, 'YData', pwm_pid_log(:,4));
            end
        
            % 3) 车体速度跟踪
            if isvalid(h_vx_ref)
                set(h_vx_ref,  'XData', pid_time_log, 'YData', chassis_speed_log(:,1));
                set(h_vy_ref,  'XData', pid_time_log, 'YData', chassis_speed_log(:,2));
                set(h_vx_real, 'XData', pid_time_log, 'YData', chassis_speed_log(:,3));
                set(h_vy_real, 'XData', pid_time_log, 'YData', chassis_speed_log(:,4));
            end
        
            % 4) 四轮速度误差
            if isvalid(h_speed_error_1)
                speed_error_log = wheel_ref_log - wheel_actual_log;
        
                set(h_speed_error_1, 'XData', pid_time_log, 'YData', speed_error_log(:,1));
                set(h_speed_error_2, 'XData', pid_time_log, 'YData', speed_error_log(:,2));
                set(h_speed_error_3, 'XData', pid_time_log, 'YData', speed_error_log(:,3));
                set(h_speed_error_4, 'XData', pid_time_log, 'YData', speed_error_log(:,4));
            end
        end

       end
        drawnow limitrate;
        t_vis = t_vis + toc(t0)*1000;

        % --- 性能统计 ---
        if mod(frame_count, 5) == 0
            elapsed = toc(tic_loop);
            fps = frame_count / elapsed;
            n = frame_count;

            fprintf('[PERF] frame=%d FPS=%.1f | b=%d/%d pos=(%.0f,%.0f) confirmed=%d\n', ...
                    frame_count, fps, b, size(q,1), ...
                    current_position(1), current_position(2), size(confirmed_obstacles,1));
            fprintf('  ms/帧: 取帧=%5.0f 激光=%5.0f mapping=%5.0f 栅格=%4.0f 碰撞=%4.0f 控制=%4.0f 可视=%4.0f\n', ...
                    t_fetch/n, t_laser/n, t_map/n, t_grid/n, t_coll/n, t_ctrl/n, t_vis/n);
        end

    catch ME
        warning('帧处理错误: %s', char(ME.message));
        pause(0.05);
        continue;
    end
end

% Quantitative summary for Experiment 2 controller comparison.
average_fps = frame_count / max(toc(tic_loop), eps);
navigation_time = control_time_elapsed;
num_methods = num_controllers;
wheel_rmse_by_method = zeros(num_methods, 1);
wheel_max_error_by_method = zeros(num_methods, 1);
pwm_saturation_by_method = zeros(num_methods, 1);
final_error_by_method = zeros(num_methods, 1);

for ctrl_i = 1:num_methods
    wheel_cols = (ctrl_i-1)*4 + (1:4);
    wheel_error = cmp_wheel_ref_log - cmp_wheel_actual_log(:, wheel_cols);
    wheel_rmse_by_method(ctrl_i) = sqrt(mean(wheel_error(:).^2));
    wheel_max_error_by_method(ctrl_i) = max(abs(wheel_error(:)));
    pwm_sat_mask = abs(cmp_pwm_log(:, wheel_cols)) >= 99;
    pwm_saturation_by_method(ctrl_i) = mean(pwm_sat_mask(:)) * 100;
    final_error_by_method(ctrl_i) = norm(cmp_position(ctrl_i,:) - goal_point);
end

% Keep these scalar names compatible with the active controller summary.
wheel_rmse = wheel_rmse_by_method(active_controller_idx);
wheel_max_error = wheel_max_error_by_method(active_controller_idx);
pwm_saturation_ratio = pwm_saturation_by_method(active_controller_idx);
final_position_error = final_error_by_method(active_controller_idx);

fprintf('\n================ Experiment 2: Motor Control Performance ================\n');
fprintf('Same planned path reference is used for all three controllers.\n');
fprintf('Active controller driving Unity: %s\n', char(CONTROLLER_NAMES(active_controller_idx)));
fprintf('Motor gain scale: [%.2f %.2f %.2f %.2f], disturbance std: %.1f mm/s\n', ...
    MOTOR_GAIN_ERROR_SCALE(1), MOTOR_GAIN_ERROR_SCALE(2), ...
    MOTOR_GAIN_ERROR_SCALE(3), MOTOR_GAIN_ERROR_SCALE(4), MOTOR_DISTURBANCE_STD);
fprintf('%-24s %14s %14s %14s %16s %16s %10s\n', ...
    'Controller', 'Wheel RMSE', 'Max Error', 'Final Error', 'PWM Saturation', 'Navigation Time', 'FPS');
for ctrl_i = 1:num_methods
    fprintf('%-24s %10.1f mm/s %10.1f mm/s %10.1f mm %12.2f%% %13.2f s %10.2f\n', ...
        char(CONTROLLER_NAMES(ctrl_i)), ...
        wheel_rmse_by_method(ctrl_i), wheel_max_error_by_method(ctrl_i), ...
        final_error_by_method(ctrl_i), pwm_saturation_by_method(ctrl_i), ...
        navigation_time, average_fps);
end
fprintf('=========================================================================\n\n');

% Plot comparison data for the three controllers.
if ~isempty(cmp_time_log)
    method_labels = cellstr(CONTROLLER_NAMES);
    t_cmp = cmp_time_log;

    figure('Name', 'Experiment 2 Controller Comparison', 'Color', 'w');
    wheel_line_styles = {'-', '--', ':', '-.'};
    wheel_colors = [0.00 0.00 0.00;
                    0.00 0.25 1.00;
                    1.00 0.00 0.00;
                    0.20 0.85 0.20];

    for ctrl_i = 1:num_methods
        subplot(num_methods, 1, ctrl_i);
        hold on;
        wheel_cols = (ctrl_i-1)*4 + (1:4);
        for wheel_i = 1:4
            plot(t_cmp, cmp_wheel_actual_log(:, wheel_cols(wheel_i)) / 10, ...
                'Color', wheel_colors(wheel_i,:), ...
                'LineStyle', wheel_line_styles{wheel_i}, ...
                'LineWidth', 1.6);
        end
        grid on; box on;
        ylabel('Wheel speed / mm/s');
        title(sprintf('(%c) %s', char('a' + ctrl_i - 1), method_labels{ctrl_i}));
        legend({'M1', 'M2', 'M3', 'M4'}, 'Location', 'best');
        if ctrl_i == num_methods
            xlabel('Time / s');
        end
    end

    set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');
    set(findall(gcf, '-property', 'FontSize'), 'FontSize', 14);
end
if ~isempty(cmp_position_log)
    figure('Name', 'Experiment 2 Actual Trajectory Comparison', 'Color', 'w');
    hold on;
    method_colors = lines(num_methods);
    method_labels = cellstr(CONTROLLER_NAMES);

    if exist('confirmed_obstacles', 'var') && ~isempty(confirmed_obstacles)
        scatter(confirmed_obstacles(:,1), confirmed_obstacles(:,2), 10, ...
            [0.65 0.65 0.65], 'filled', 'DisplayName', 'Confirmed obstacles');
    end

    if exist('reference_path_log', 'var') && ~isempty(reference_path_log)
        plot(reference_path_log(:,1), reference_path_log(:,2), 'k--', 'LineWidth', 1.8, ...
            'DisplayName', 'Reference path');
    end

    for ctrl_i = 1:num_methods
        pos_cols = (ctrl_i-1)*2 + (1:2);
        plot(cmp_position_log(:, pos_cols(1)), cmp_position_log(:, pos_cols(2)), ...
            'Color', method_colors(ctrl_i,:), 'LineWidth', 2.0, ...
            'DisplayName', method_labels{ctrl_i});
    end

    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, ...
        'LineWidth', 2, 'DisplayName', 'Start');
    plot(goal_point(1), goal_point(2), 'rp', 'MarkerSize', 13, ...
        'LineWidth', 2, 'DisplayName', 'Goal');

    axis equal;
    grid on; box on;
    xlabel('X / mm');
    ylabel('Y / mm');
    title('Actual Trajectory Comparison');
    legend('Location', 'best');
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 14, 'LineWidth', 1.2);
end

% Record and display the final stopped state before leaving the script.
stop_time = control_time_elapsed + CONTROL_DT_NOMINAL;
pid_time_log = [pid_time_log; stop_time];
wheel_ref_log = [wheel_ref_log; zeros(1,4)];
wheel_actual_log = [wheel_actual_log; zeros(1,4)];
pwm_log = [pwm_log; zeros(1,4)];
pwm_ff_log = [pwm_ff_log; zeros(1,4)];
pwm_pid_log = [pwm_pid_log; zeros(1,4)];
chassis_speed_log = [chassis_speed_log; zeros(1,4)];

set(h_wheel_ref_1, 'XData', pid_time_log, 'YData', wheel_ref_log(:,1));
set(h_wheel_act_1, 'XData', pid_time_log, 'YData', wheel_actual_log(:,1));
set(h_wheel_ref_2, 'XData', pid_time_log, 'YData', wheel_ref_log(:,2));
set(h_wheel_act_2, 'XData', pid_time_log, 'YData', wheel_actual_log(:,2));
set(h_wheel_ref_3, 'XData', pid_time_log, 'YData', wheel_ref_log(:,3));
set(h_wheel_act_3, 'XData', pid_time_log, 'YData', wheel_actual_log(:,3));
set(h_wheel_ref_4, 'XData', pid_time_log, 'YData', wheel_ref_log(:,4));
set(h_wheel_act_4, 'XData', pid_time_log, 'YData', wheel_actual_log(:,4));

set(h_pwm_1, 'XData', pid_time_log, 'YData', pwm_log(:,1));
set(h_pwm_2, 'XData', pid_time_log, 'YData', pwm_log(:,2));
set(h_pwm_3, 'XData', pid_time_log, 'YData', pwm_log(:,3));
set(h_pwm_4, 'XData', pid_time_log, 'YData', pwm_log(:,4));

set(h_effort_ff, 'XData', pid_time_log, 'YData', mean(abs(pwm_ff_log), 2));
set(h_effort_pid, 'XData', pid_time_log, 'YData', mean(abs(pwm_pid_log), 2));
set(h_effort_total, 'XData', pid_time_log, 'YData', mean(abs(pwm_log), 2));
set(h_pid_corr_1, 'XData', pid_time_log, 'YData', pwm_pid_log(:,1));
set(h_pid_corr_2, 'XData', pid_time_log, 'YData', pwm_pid_log(:,2));
set(h_pid_corr_3, 'XData', pid_time_log, 'YData', pwm_pid_log(:,3));
set(h_pid_corr_4, 'XData', pid_time_log, 'YData', pwm_pid_log(:,4));

set(h_vx_ref,  'XData', pid_time_log, 'YData', chassis_speed_log(:,1));
set(h_vy_ref,  'XData', pid_time_log, 'YData', chassis_speed_log(:,2));
set(h_vx_real, 'XData', pid_time_log, 'YData', chassis_speed_log(:,3));
set(h_vy_real, 'XData', pid_time_log, 'YData', chassis_speed_log(:,4));

speed_error_log = wheel_ref_log - wheel_actual_log;
set(h_speed_error_1, 'XData', pid_time_log, 'YData', speed_error_log(:,1));
set(h_speed_error_2, 'XData', pid_time_log, 'YData', speed_error_log(:,2));
set(h_speed_error_3, 'XData', pid_time_log, 'YData', speed_error_log(:,3));
set(h_speed_error_4, 'XData', pid_time_log, 'YData', speed_error_log(:,4));
drawnow;


function motor_speed = simulateTTMotorWithGainError(motor_speed, pwm_cmd, dt, tau, gain_actual, max_speed)
%SIMULATETTMOTORWITHGAINERROR First-order motor model with wheel-dependent gain.
    target_speed = gain_actual(:) .* pwm_cmd(:);
    response_ratio = 1 - exp(-dt / max(tau, 1e-3));
    motor_speed = motor_speed + (target_speed - motor_speed) * response_ratio;
    motor_speed = max(min(motor_speed, max_speed), -max_speed);
end

function path_len = computePathLength(path_points)
%COMPUTEPATHLENGTH Return the accumulated length of a 2-D polyline.
    if isempty(path_points) || size(path_points, 1) < 2
        path_len = 0;
        return;
    end
    path_len = sum(vecnorm(diff(path_points, 1, 1), 2, 2));
end

% ============================================================
% ================== 麦轮可执行路径规划函数 ==================
% ============================================================

function [raw_path, q_exec] = planMecanumExecutablePath(...
        start_pos, goal_pos, obstacles, safety_dist, exec_step_len)
%PLANMECANUMEXECUTABLEPATH
% 先调用原 RRT* 生成原始路径，再转换为麦轮小车 8方向可执行轨迹。

    obstacles = safeObstacles(obstacles);

    raw_path = rrt_star_path_planning2(start_pos, goal_pos, obstacles, safety_dist);
    q_exec = buildMecanumExecutablePath(raw_path, exec_step_len);

    if isempty(q_exec) || size(q_exec,1) < 2
        error('[PLAN] Failed to generate mecanum executable path.');
    end
end

function [raw_path, q_exec] = tryPlanMecanumPath(...
        start_pos, goal_pos, obstacles, safety_dist_list, exec_step_len)
%TRYPLANMECANUMPATH
% 按多个安全距离尝试规划，并把成功路径转换为麦轮 8方向可执行轨迹。

    raw_path = [];
    q_exec = [];
    obstacles = safeObstacles(obstacles);

    for sd = safety_dist_list
        try
            raw_path = rrt_star_path_planning2(start_pos, goal_pos, obstacles, sd);
            q_exec = buildMecanumExecutablePath(raw_path, exec_step_len);

            if ~isempty(q_exec) && size(q_exec,1) >= 2
                fprintf('[PLAN] 成功，安全距离=%.0fmm，可执行点数=%d\n', ...
                        sd, size(q_exec,1));
                return;
            end
        catch ME
            fprintf('[PLAN] 安全距离=%.0fmm 失败：%s\n', sd, ME.message);
        end
    end

    raw_path = [];
    q_exec = [];
end

function q_exec = buildMecanumExecutablePath(q_raw, step_len)
%BUILDMECANUMEXECUTABLEPATH
% 将 RRT* 输出的任意二维路径转换为麦轮小车可执行轨迹。
%
% 坐标约定：
% X 正方向 = 小车右移
% X 负方向 = 小车左移
% Y 正方向 = 小车前进
% Y 负方向 = 小车后退
%
% 输出 q_exec 中每两个相邻点之间只允许：
% [0,+d]、[0,-d]、[+d,0]、[-d,0]、
% [+d,+d]、[-d,+d]、[+d,-d]、[-d,-d]。

    if isempty(q_raw) || size(q_raw,1) < 2
        q_exec = q_raw;
        return;
    end

    q_exec = q_raw(1,:);

    for i = 1:size(q_raw,1)-1
        p0 = q_exec(end,:);
        p1 = q_raw(i+1,:);

        segment_exec = decomposeSegmentToMecanumDirs(p0, p1, step_len);

        if ~isempty(segment_exec)
            q_exec = [q_exec; segment_exec]; %#ok<AGROW>
        end
    end

    q_exec = removeNearDuplicatePoints(q_exec, 1e-6);
end

function pts = decomposeSegmentToMecanumDirs(p0, p1, step_len)
%DECOMPOSESEGMENTTOMECANUMDIRS
% 将任意线段 p0 -> p1 分解为麦轮 8方向运动段。
%
% 分解原则：
% 1) 先走斜向部分，消耗 min(abs(dx), abs(dy))；
% 2) 再走剩余的 X 或 Y 方向；
% 3) 每个小段长度不超过 step_len。
%
% 示例：
% [0,0] -> [1000,500]
% 会变为：
% 右前 500mm + 右移 500mm。

    dx = p1(1) - p0(1);
    dy = p1(2) - p0(2);

    sx = sign(dx);
    sy = sign(dy);

    ax = abs(dx);
    ay = abs(dy);

    pts = [];
    p = p0;

    diag_len = min(ax, ay);

    while diag_len > 1e-6
        d = min(step_len, diag_len);
        p = p + [sx*d, sy*d];
        pts = [pts; p]; %#ok<AGROW>
        diag_len = diag_len - d;
    end

    remain_x = ax - min(ax, ay);

    while remain_x > 1e-6
        d = min(step_len, remain_x);
        p = p + [sx*d, 0];
        pts = [pts; p]; %#ok<AGROW>
        remain_x = remain_x - d;
    end

    remain_y = ay - min(ax, ay);

    while remain_y > 1e-6
        d = min(step_len, remain_y);
        p = p + [0, sy*d];
        pts = [pts; p]; %#ok<AGROW>
        remain_y = remain_y - d;
    end
end

function q2 = removeNearDuplicatePoints(q, threshold)
%REMOVENEARDUPLICATEPOINTS 删除过近重复点。

    if isempty(q)
        q2 = q;
        return;
    end

    q2 = q(1,:);

    for i = 2:size(q,1)
        if norm(q(i,:) - q2(end,:)) > threshold
            q2 = [q2; q(i,:)]; %#ok<AGROW>
        end
    end
end

function ok = checkMecanumExecutablePath(q)
%CHECKMECANUMEXECUTABLEPATH
% 检查 q 中每个相邻点段是否满足麦轮 8方向运动约束。

    ok = true;

    if isempty(q) || size(q,1) < 2
        warning('[CHECK] q is empty or has insufficient points.');
        ok = false;
        return;
    end

    dq = diff(q, 1, 1);

    for i = 1:size(dq,1)
        dx = dq(i,1);
        dy = dq(i,2);

        if abs(dx) < 1e-6
            dx = 0;
        end

        if abs(dy) < 1e-6
            dy = 0;
        end

        is_forward_back = dx == 0 && dy ~= 0;
        is_left_right   = dx ~= 0 && dy == 0;
        is_diagonal     = dx ~= 0 && dy ~= 0 && abs(abs(dx) - abs(dy)) < 1e-6;

        if ~(is_forward_back || is_left_right || is_diagonal)
            ok = false;
            fprintf('[CHECK ERROR] 第 %d 段不可执行: dx=%.3f, dy=%.3f\n', ...
                    i, dx, dy);
        end
    end

    if ok
        fprintf('[CHECK] q satisfies mecanum 8-direction constraint, points=%d.\n', size(q,1));
    else
        warning('[CHECK] q contains non-executable direction segments.');
    end
end


% ============================================================
% ===================== 栅格地图相关函数 =====================
% ============================================================

function [grid_hit, grid_seen] = updateGridMap(grid_hit, grid_seen, xy_pts, ...
                                                origin, resolution, grid_size, frame)
%UPDATEGRIDMAP
% 将世界坐标障碍物点云量化到栅格，命中计数 +1，并记录最后命中帧号。

    if isempty(xy_pts)
        return;
    end

    ix = floor((xy_pts(:,1) - origin(1)) / resolution) + 1;
    iy = floor((xy_pts(:,2) - origin(2)) / resolution) + 1;

    valid = ix >= 1 & ix <= grid_size(1) & iy >= 1 & iy <= grid_size(2);
    ix = ix(valid);
    iy = iy(valid);

    if isempty(ix)
        return;
    end

    cells = unique([ix, iy], 'rows');
    lin_idx = sub2ind([grid_size(1), grid_size(2)], cells(:,1), cells(:,2));

    grid_hit(lin_idx) = min(int16(grid_hit(lin_idx)) + int16(1), int16(32000));
    grid_seen(lin_idx) = int32(frame);
end

function grid_hit = decayGridMap(grid_hit, grid_seen, current_frame, decay_frames)
%DECAYGRIDMAP
% 长时间未命中的障碍物栅格逐步衰减。

    stale_mask = (grid_hit > 0) & ...
                 (int32(current_frame) - grid_seen > int32(decay_frames));
    grid_hit(stale_mask) = grid_hit(stale_mask) - int16(1);
end

function obstacles = extractConfirmedObstacles(grid_hit, origin, resolution, threshold)
%EXTRACTCONFIRMEDOBSTACLES
% 提取命中数 >= threshold 的栅格中心作为确认障碍物点。

    [ix, iy] = find(grid_hit >= int16(threshold));

    if isempty(ix)
        obstacles = zeros(0, 2);
        return;
    end

    x = origin(1) + (double(ix(:)) - 0.5) * resolution;
    y = origin(2) + (double(iy(:)) - 0.5) * resolution;

    obstacles = [x, y];
end


% ============================================================
% ====================== 局部规划辅助函数 ====================
% ============================================================

function obs = safeObstacles(obs)
%SAFEOBSTACLES
% 确保障碍物矩阵始终为 N×2 数值矩阵。

    if isempty(obs)
        obs = zeros(0, 2);
        return;
    end

    if isvector(obs)
        obs = obs(:);
        if size(obs, 2) == 1
            obs = [obs, zeros(size(obs,1), 1)];
        end
    end

    if size(obs, 2) ~= 2
        obs = zeros(0, 2);
    end
end

function sub_goal = computeSubGoal(current_pos, real_goal, local_radius)
%COMPUTESUBGOAL
% 计算局部规划子目标。

    delta = real_goal - current_pos;
    dist = norm(delta);

    if dist <= local_radius
        sub_goal = real_goal;
    else
        sub_goal = current_pos + delta / dist * local_radius;
    end
end

function local_obs = filterLocalObstacles(all_obs, center, radius)
%FILTERLOCALOBSTACLES
% 仅保留当前位置附近指定半径内的障碍物。

    if isempty(all_obs)
        local_obs = zeros(0,2);
        return;
    end

    dx = all_obs(:,1) - center(1);
    dy = all_obs(:,2) - center(2);

    in_range = (dx.^2 + dy.^2) <= radius^2;
    local_obs = all_obs(in_range, :);
end


% ============================================================
% ======================= 图像容错函数 =======================
% ============================================================

function ok = isValidImageFrame(img)
%ISVALIDIMAGEFRAME
% 判断从 TCP 读取到的图像是否有效，避免坏 PNG / 空帧进入后续算法。

    ok = false;

    if isempty(img)
        return;
    end

    if ~isnumeric(img) && ~islogical(img)
        return;
    end

    if ndims(img) < 2
        return;
    end

    h = size(img, 1);
    w = size(img, 2);

    if h < 10 || w < 10
        return;
    end

    if isnumeric(img)
        if all(isnan(img(:)))
            return;
        end
    end

    ok = true;
end















