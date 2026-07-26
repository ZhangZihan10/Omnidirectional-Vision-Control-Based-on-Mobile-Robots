%% 麦轮小车可执行轨迹版本：栅格地图剪枝 + 局部RRT* + 8方向运动约束
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
safety_distance = 1700;

% ===== 实时性参数 =====
VIS_DECIMATE_GLOBAL   = 5;
OBSTACLE_EXTRACT_FREQ = 5;
COLLISION_CHECK_FREQ  = 5;

% ===== 栅格地图参数 =====
GRID_RESOLUTION = 150;
GRID_ORIGIN     = [-15000, -15000];
GRID_SIZE       = [400, 200];

HIT_THRESHOLD  = 2;
DECAY_FRAMES   = 60;
DECAY_INTERVAL = 10;

% ===== 局部 RRT* 参数 =====
LOCAL_PLAN_RADIUS    = 6000;
REPLAN_COOLDOWN      = 10;
SAFETY_DIST_FALLBACK = [safety_distance, safety_distance*0.7, safety_distance*0.5];

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

confirmed_obstacles = zeros(0, 2);

% ===== 本版本使用同步取帧，避免异步 TCP 图像流错位导致 PNG Read Error =====
fprintf('[INFO] 使用同步取帧模式，不启用 parfeval 异步预取\n');

frame_count = 0;
replan_cooldown_counter = 0;

t_fetch = 0; t_laser = 0; t_map = 0;
t_grid  = 0; t_coll  = 0; t_ctrl = 0; t_vis = 0;
tic_loop = tic;

% ===== 主循环 =====
while true
    try
        frame_count = frame_count + 1;

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
            warning('[LASER] black_image 无效，跳过本帧');
            continue;
        end

        try
            binaryImage = las_segm1(black_image);
        catch ME_laser
            warning('[LASER] 激光提取失败: %s，跳过本帧', char(ME_fetch.message));
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

        if mod(frame_count, OBSTACLE_EXTRACT_FREQ) == 0 || isempty(confirmed_obstacles)
            confirmed_obstacles = extractConfirmedObstacles(...
                grid_hit_count, GRID_ORIGIN, GRID_RESOLUTION, HIT_THRESHOLD);
        end
        t_grid = t_grid + toc(t0)*1000;

        % --- Step 5: 碰撞检测 + 局部重规划 ---
        t0 = tic;
        replan_cooldown_counter = max(0, replan_cooldown_counter - 1);

        need_replan = false;
        if replan_cooldown_counter == 0 && ...
           ~isempty(confirmed_obstacles) && ...
           mod(frame_count, COLLISION_CHECK_FREQ) == 0

            % 注意：这里检测的是麦轮可执行轨迹 smoothed_path，不再检测平滑曲线。
            need_replan = check_path_collision(smoothed_path, confirmed_obstacles, safety_distance);
        end

        if need_replan
            disp('麦轮可执行轨迹与确认障碍物冲突，启动局部重规划...');

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
                smoothed_path = q;
                checkMecanumExecutablePath(q);
                b = 1;
                replan_cooldown_counter = REPLAN_COOLDOWN;
            else
                warning('[REPLAN] 所有安全距离均失败，保持当前可执行轨迹继续执行');
                replan_cooldown_counter = REPLAN_COOLDOWN;
            end
        end
        t_coll = t_coll + toc(t0)*1000;

        % --- Step 6: 运动控制 / 发送位置到 Unity ---
        % 注意：q 已经是麦轮 8方向可执行轨迹。
        % 这里保留原程序的 func_Car(Client,q,b)，将 q(b,:) 发送给 Unity，
        % 让 Unity 端虚拟小车真正沿着可执行轨迹点运动。
        t0 = tic;
        if b <= size(q, 1)
        
            % 1) 发送当前目标轨迹点到 Unity
            func_Car(Client, q, b);
        
            % 2) MATLAB 端同步更新当前位置，用于下一帧 mapping 和重规划
            current_position = q(b,:);
            robot_trace = [robot_trace; current_position];
            cvsyst_x = q(b,1);
            cvsyst_y = q(b,2);
        
            % 3) 执行下一个轨迹点
            b = b + 1;
        
            % 4) 接近当前子目标末端，但还没到最终目标时，继续规划下一段
            dist_to_goal = norm(current_position - goal_point);
            dist_to_subgoal_end = norm(current_position - q(end,:));
        
            if dist_to_subgoal_end < 300 && dist_to_goal > 500
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
                    smoothed_path = q;
        
                    if isempty(q) || size(q,1) < 2
                        error('[PATH] q 为空或路径点不足，无法继续');
                    end
        
                    fprintf('[PATH] 新可执行轨迹点数 = %d\n', size(q,1));
                    checkMecanumExecutablePath(q);
                    b = 1;
                    replan_cooldown_counter = REPLAN_COOLDOWN;
                end
            end
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
                    smoothed_path = q;
        
                    if isempty(q) || size(q,1) < 2
                        error('[PATH] q 为空或路径点不足，无法继续');
                    end
        
                    fprintf('[PATH] 新可执行轨迹点数 = %d\n', size(q,1));
                    checkMecanumExecutablePath(q);
                    b = 1;
                    replan_cooldown_counter = REPLAN_COOLDOWN;
                end
            else
                fprintf('[NAV] 已到达终点，停止规划。\n');
            end
        end
        t_ctrl = t_ctrl + toc(t0)*1000;

        % --- Step 7: 可视化 ---
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


% ============================================================
% ================== 麦轮可执行路径规划函数 ==================
% ============================================================

function [raw_path, q_exec] = planMecanumExecutablePath(...
        start_pos, goal_pos, obstacles, safety_dist, exec_step_len)
%PLANMECANUMEXECUTABLEPATH
% 先调用原 RRT* 生成原始路径，再转换为麦轮小车 8方向可执行轨迹。

    obstacles = safeObstacles(obstacles);

    raw_path = rrt_star_path_planning(start_pos, goal_pos, obstacles, safety_dist);
    q_exec = buildMecanumExecutablePath(raw_path, exec_step_len);

    if isempty(q_exec) || size(q_exec,1) < 2
        error('[PLAN] 麦轮可执行轨迹生成失败');
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
            raw_path = rrt_star_path_planning(start_pos, goal_pos, obstacles, sd);
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
        warning('[CHECK] q 为空或点数不足');
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
        fprintf('[CHECK] q 满足麦轮小车 8方向运动约束，路径点数=%d。\n', size(q,1));
    else
        warning('[CHECK] q 存在不可执行方向，请检查路径转换函数');
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
