%% 准确性优化版本：栅格地图剪枝 + 局部RRT* + 全向运动学约束
%% 在 realtime_obstacle_avoidance_optimized.m 基础上叠加准确性优化
%% 适用于：全向移动机器人

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

% 起点 / 终点
start_point = [0, 0];
goal_point  = [18000, 50];

robot_trace      = [];
safety_distance  = 1200;

% ===== 实时性参数 =====
VIS_DECIMATE_GLOBAL    = 5;    % 全局地图每 N 帧刷新一次
ASYNC_TIMEOUT          = 0.3;  % 异步取帧超时（秒），超时立即降级同步，不要设太大
OBSTACLE_EXTRACT_FREQ  = 5;    % 每 N 帧提取一次确认障碍物（避免每帧全量 find）
COLLISION_CHECK_FREQ   = 5;    % 每 N 帧做一次碰撞检测（与提取频率对齐）

% ===== 准确性参数（核心调参点）=====
% --- 优化1：栅格地图剪枝 ---
GRID_RESOLUTION   = 150;     % 栅格分辨率 (mm/格)
% 工作区覆盖：x[-15000~45000], y[-15000~15000]
% x 方向需要覆盖 goal_point(1)=18000，故取 45000 留足余量
% x格数 = (45000-(-15000))/150 = 400，y格数 = (15000-(-15000))/150 = 200
GRID_ORIGIN       = [-15000, -15000];
GRID_SIZE         = [400, 200];   % [x方向格数, y方向格数]，覆盖x:45000mm y:30000mm
HIT_THRESHOLD     = 1;       % 命中≥N次才确认为障碍物
DECAY_SECONDS     = 2.0;     % 超过 N 秒未被命中则直接清零（与帧率无关）
DECAY_INTERVAL    = 4;      % 每 N 帧执行一次衰减检查

% --- 优化2：局部 RRT* 规划域 ---
LOCAL_PLAN_RADIUS    = 6000;  % 局部规划半径 (mm)
REPLAN_COOLDOWN      = 10;    % 重规划冷却帧数，避免每帧都触发
% 渐进式安全距离：找不到路时依次尝试以下值（单位mm）
SAFETY_DIST_FALLBACK = [safety_distance, safety_distance*0.7, safety_distance*0.5];

% --- 优化3：全向机器人运动学约束 ---
MAX_SEGMENT_LENGTH = 1500;   % 路径段最大长度 (mm)，500太小导致插入过多中间点
SMOOTH_RESAMPLE_N  = 30;     % B样条重采样点数，50对于实时控制过密
MAX_ACCELERATION   = 2000;   % 最大加速度等效约束 (mm/s²)

% ===== 初始化栅格地图 =====
% grid_hit_count(i,j)  = 该栅格累计被激光命中次数
% grid_last_seen(i,j)  = 该栅格最后一次被命中的时间戳（秒，相对于程序启动）
grid_hit_count  = zeros(GRID_SIZE(1), GRID_SIZE(2), 'int16');
grid_last_seen  = zeros(GRID_SIZE(1), GRID_SIZE(2), 'single');  % 秒，而非帧号
t_start = tic;  % 程序启动时间基准

% ===== 第一帧 =====
black_image = ImageReadTCP_One(Client, 'Center');
binaryImage = las_segm1(black_image);

% ===== 创建图像句柄 =====
figure;
f1 = subplot(1,4,1); h1 = imshow(black_image);  title('Original Image');
f2 = subplot(1,4,2); h2 = imshow(binaryImage);  title('Laser Extraction');

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
h_path      = plot(nan, nan, 'm--', 'LineWidth', 1.0);  % 当前规划路径
title('Global Map with Real Path & Plan');
xlabel('X/mm'); ylabel('Y/mm'); grid on;
xlim([-10000, 20000]); ylim([-10000, 20000]);
legend({'Confirmed Obstacles', 'Robot Path', 'Planned Path'}, 'Location', 'best');

% ===== 初始路径规划 =====
b = 1;
cvsyst_x = start_point(1);
cvsyst_y = start_point(2);

[x1_init, y1_init] = mapping(binaryImage, cvsyst_rot, 0, 0, ...
                              camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
if size(x1_init, 2) > 1
    x1_init(:,1) = []; y1_init(:,1) = [];
end
x2_init = x1_init + cvsyst_x;
y2_init = y1_init + cvsyst_y;
xy2 = [x2_init(:), y2_init(:)];

% 初次更新栅格地图
[grid_hit_count, grid_last_seen] = updateGridMap(...
    grid_hit_count, grid_last_seen, xy2, ...
    GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, toc(t_start));

% 初次规划：直接用第一帧原始点云（不依赖 HIT_THRESHOLD）
% 原因：栅格命中确认需要多帧积累，第一帧 HIT_THRESHOLD≥2 会导致障碍物为空
% 主循环启动后才切换为确认障碍物体系
init_obstacles = safeObstacles([x2_init(:), y2_init(:)]);

% 诊断打印，确认规划输入合理
fprintf('[INIT] 起点=(%.0f,%.0f) 终点=(%.0f,%.0f)\n', ...
        start_point(1), start_point(2), goal_point(1), goal_point(2));
fprintf('[INIT] 初始障碍物点数=%d，安全距离=%.0fmm\n', ...
        size(init_obstacles,1), safety_distance);
fprintf('[INIT] 障碍物 x 范围: [%.0f, %.0f]\n', ...
        min(init_obstacles(:,1)), max(init_obstacles(:,1)));
fprintf('[INIT] 障碍物 y 范围: [%.0f, %.0f]\n', ...
        min(init_obstacles(:,2)), max(init_obstacles(:,2)));

% 初次规划：用全局规划（确认起点到终点可达）
path = rrt_star_path_planning(start_point, goal_point, ...
                              init_obstacles, safety_distance);
% 用运动学约束的平滑器
smoothed_path = smoothPathOmni(path, MAX_SEGMENT_LENGTH, ...
                                SMOOTH_RESAMPLE_N, MAX_ACCELERATION);

% 可视化初始规划
figure;
if ~isempty(init_obstacles)
    scatter(init_obstacles(:,1), init_obstacles(:,2), ...
            'filled', 'red'); hold on;
end
plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal_point(1),  goal_point(2),  'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(smoothed_path(:,1), smoothed_path(:,2), 'k-', 'LineWidth', 2);
axis equal; xlabel('X/mm'); ylabel('Y/mm');
title('Initial Path Planning (with Grid Pruning)'); hold off;

q = smoothed_path;
cvsyst_x = q(b,1); cvsyst_y = q(b,2);
current_position = q(b,:);

% confirmed_obstacles 从空开始，由主循环栅格体系逐帧积累
% 不继承 init_obstacles（原始点云含噪点，会导致主循环碰撞误判）
confirmed_obstacles = zeros(0, 2);

% ===== 启动异步预取 =====
hasParallel = ~isempty(ver('parallel'));
if hasParallel
    pool = gcp('nocreate');
    if isempty(pool)
        parpool('local', 1);
    end
    fut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    fprintf('[INFO] 异步预取模式已启用 (parfeval)\n');
else
    fut = [];
    fprintf('[WARN] 未检测到 Parallel Toolbox，回退同步模式\n');
end

frame_count  = 0;
replan_cooldown_counter = 0;
% 逐步骤累计耗时 (ms)
t_fetch = 0; t_laser = 0; t_map = 0;
t_grid  = 0; t_coll  = 0; t_ctrl = 0; t_vis = 0;
tic_loop = tic;

% ===== 主循环 =====
while true
    try
        frame_count = frame_count + 1;

        % --- Step 1: 异步取帧 ---
        t0 = tic;
        [black_image, fut] = safeFetch(fut, Client, hasParallel, ASYNC_TIMEOUT);
        t_fetch = t_fetch + toc(t0)*1000;

        % --- Step 2: 激光提取 ---
        t0 = tic;
        binaryImage = las_segm1(black_image);
        t_laser = t_laser + toc(t0)*1000;

        % --- Step 3: Mapping ---
        t0 = tic;
        [x1, y1] = mapping(binaryImage, cvsyst_rot, 0, 0, ...
                           camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
        if size(x1, 2) > 1
            x1(:,1) = []; y1(:,1) = [];
        end
        x2 = x1 + cvsyst_x;
        y2 = y1 + cvsyst_y;
        xy2 = [x2(:), y2(:)];
        t_map = t_map + toc(t0)*1000;
        xy2 = [x2(:), y2(:)];

        % --- Step 4: 栅格地图更新 ---
        t0 = tic;
        current_time = toc(t_start);  % 当前时间戳（秒）

        [grid_hit_count, grid_last_seen] = updateGridMap(...
            grid_hit_count, grid_last_seen, xy2, ...
            GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, current_time);

        % 定期衰减：超过 DECAY_SECONDS 未被命中的栅格直接清零
        if mod(frame_count, DECAY_INTERVAL) == 0
            grid_hit_count = decayGridMap(...
                grid_hit_count, grid_last_seen, current_time, DECAY_SECONDS);
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
            need_replan = check_path_collision(smoothed_path, confirmed_obstacles, safety_distance);
        end

        if need_replan
            disp('路径与确认障碍物冲突，启动局部重规划...');
            sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
            local_obstacles = filterLocalObstacles(...
                confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
            local_obstacles = safeObstacles(local_obstacles);

            new_path = [];
            for sd = SAFETY_DIST_FALLBACK
                try
                    new_path = rrt_star_path_planning(current_position, sub_goal, ...
                                                      local_obstacles, sd);
                    fprintf('[REPLAN] 局部规划成功，安全距离=%.0fmm\n', sd);
                    break;
                catch
                    fprintf('[REPLAN] 安全距离=%.0fmm 找不到路，尝试更小值...\n', sd);
                end
            end

            if ~isempty(new_path)
                smoothed_path = smoothPathOmni(new_path, MAX_SEGMENT_LENGTH, ...
                                               SMOOTH_RESAMPLE_N, MAX_ACCELERATION);
                q = smoothed_path;
                b = 1;
                replan_cooldown_counter = REPLAN_COOLDOWN;
            else
                warning('[REPLAN] 所有安全距离均失败，保持当前路径继续执行');
                replan_cooldown_counter = REPLAN_COOLDOWN;
            end
        end
        t_coll = t_coll + toc(t0)*1000;

        % --- Step 6: 运动控制 ---
        t0 = tic;
        if b <= size(q, 1)
            func_Car(Client, q, b);
            current_position = q(b,:);
            robot_trace = [robot_trace; current_position];
            cvsyst_x = q(b,1);
            cvsyst_y = q(b,2);
            b = b + 1;

            dist_to_goal        = norm(current_position - goal_point);
            dist_to_subgoal_end = norm(current_position - q(end,:));
            if dist_to_subgoal_end < 300 && dist_to_goal > 500
                disp('接近子目标，规划下一段...');
                sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
                local_obstacles = filterLocalObstacles(...
                    confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
                local_obstacles = safeObstacles(local_obstacles);
                new_path = [];
                for sd = SAFETY_DIST_FALLBACK
                    try
                        new_path = rrt_star_path_planning(current_position, sub_goal, ...
                                                          local_obstacles, sd);
                        break;
                    catch
                    end
                end
                if ~isempty(new_path)
                    smoothed_path = smoothPathOmni(new_path, MAX_SEGMENT_LENGTH, ...
                                                   SMOOTH_RESAMPLE_N, MAX_ACCELERATION);
                    q = smoothed_path; b = 1;
                    replan_cooldown_counter = REPLAN_COOLDOWN;
                end
            end
        else
            dist_to_goal = norm(current_position - goal_point);
            if dist_to_goal > 300
                fprintf('[NAV] 路径执行完毕，距终点%.0fmm，规划下一段...\n', dist_to_goal);
                sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
                local_obstacles = filterLocalObstacles(...
                    confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS);
                local_obstacles = safeObstacles(local_obstacles);
                new_path = [];
                for sd = SAFETY_DIST_FALLBACK
                    try
                        new_path = rrt_star_path_planning(current_position, sub_goal, ...
                                                          local_obstacles, sd);
                        break;
                    catch
                    end
                end
                if ~isempty(new_path)
                    smoothed_path = smoothPathOmni(new_path, MAX_SEGMENT_LENGTH, ...
                                                   SMOOTH_RESAMPLE_N, MAX_ACCELERATION);
                    q = smoothed_path; b = 1;
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
        if mod(frame_count, VIS_DECIMATE_GLOBAL) == 0
            set(h4, 'XData', x2, 'YData', y2);
            set(h_trace, 'XData', robot_trace(:,1), 'YData', robot_trace(:,2));
            set(h_path,  'XData', smoothed_path(:,1), 'YData', smoothed_path(:,2));
            if ~isempty(confirmed_obstacles)
                set(h_globalmap, 'XData', confirmed_obstacles(:,1), ...
                                 'YData', confirmed_obstacles(:,2));
            end
        end
        drawnow limitrate;
        t_vis = t_vis + toc(t0)*1000;

        % --- 性能统计：每5帧打印各步骤平均耗时 ---
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
        fut = resetFuture(fut, Client, hasParallel);
        continue;
    end
end

% ===== 清理 =====
if hasParallel && ~isempty(fut) && isvalid(fut)
    cancel(fut);
end


% ============================================================
% ============== 准确性优化 1：栅格地图相关函数 ==============
% ============================================================

function [grid_hit, grid_seen] = updateGridMap(grid_hit, grid_seen, xy_pts, ...
                                                origin, resolution, grid_size, timestamp)
    % 把世界坐标点云量化到栅格，命中计数+1，记录命中时间戳（秒）
    if isempty(xy_pts)
        return;
    end
    ix = floor((xy_pts(:,1) - origin(1)) / resolution) + 1;
    iy = floor((xy_pts(:,2) - origin(2)) / resolution) + 1;

    valid = ix >= 1 & ix <= grid_size(1) & iy >= 1 & iy <= grid_size(2);
    ix = ix(valid); iy = iy(valid);
    if isempty(ix)
        return;
    end

    cells   = unique([ix, iy], 'rows');
    lin_idx = sub2ind([grid_size(1), grid_size(2)], cells(:,1), cells(:,2));

    grid_hit(lin_idx)  = min(int16(grid_hit(lin_idx)) + int16(1), int16(32000));
    grid_seen(lin_idx) = single(timestamp);  % 记录秒级时间戳
end

function grid_hit = decayGridMap(grid_hit, grid_seen, current_time, decay_seconds)
    % 超过 decay_seconds 秒未被命中的栅格直接清零（而非逐步衰减）
    % 修复原来逐步-1的设计缺陷：静态障碍物累计100次命中需要1000帧才能消失
    stale_mask = (grid_hit > 0) & ...
                 (single(current_time) - grid_seen > single(decay_seconds));
    grid_hit(stale_mask) = int16(0);
end

function obstacles = extractConfirmedObstacles(grid_hit, origin, resolution, threshold)
    % 提取累计命中数 >= 阈值的栅格中心作为确认障碍物
    % 输出格式：N×2 矩阵 [x, y]，严格行向量排列，与 rrt_star_path_planning 兼容
    [ix, iy] = find(grid_hit >= int16(threshold));
    if isempty(ix)
        obstacles = zeros(0, 2);   % 返回 0×2 空矩阵而非 []，避免后续维度错误
        return;
    end
    % find() 返回列向量，double() 确保后续运算不出 int16 溢出
    x = origin(1) + (double(ix(:)) - 0.5) * resolution;  % 列向量
    y = origin(2) + (double(iy(:)) - 0.5) * resolution;  % 列向量
    obstacles = [x, y];   % N×2
end

% ============================================================
% ============== 准确性优化 2：局部 RRT* 相关函数 =============
% ============================================================

function obs = safeObstacles(obs)
    % 统一校验障碍物矩阵格式，确保传入 rrt_star 的始终是 N×2 数值矩阵
    % 防止 [] / 0×0 / 列向量 等异常维度导致 check_collision 内部维度不匹配
    if isempty(obs)
        obs = zeros(0, 2);
        return;
    end
    % 若意外传入列向量（1×N），转置为行向量组（N×1 -> 补零为 N×2）
    if isvector(obs)
        obs = obs(:);           % 确保列向量
        if size(obs, 2) == 1
            obs = [obs, zeros(size(obs,1), 1)];  % 补零（不应发生，保险用）
        end
    end
    % 确保是 N×2
    if size(obs, 2) ~= 2
        obs = zeros(0, 2);
    end
end

function sub_goal = computeSubGoal(current_pos, real_goal, local_radius)
    % 把真实目标投影到以当前位置为中心、半径为 local_radius 的圆上
    % 如果真实目标已在圆内，直接返回真实目标
    delta = real_goal - current_pos;
    dist = norm(delta);
    if dist <= local_radius
        sub_goal = real_goal;
    else
        % 沿着指向目标的方向截取
        sub_goal = current_pos + delta / dist * local_radius;
    end
end

function local_obs = filterLocalObstacles(all_obs, center, radius)
    % 仅保留以 center 为中心、半径 radius 的圆形区域内的障碍物
    if isempty(all_obs)
        local_obs = [];
        return;
    end
    dx = all_obs(:,1) - center(1);
    dy = all_obs(:,2) - center(2);
    in_range = (dx.^2 + dy.^2) <= radius^2;
    local_obs = all_obs(in_range, :);
end

% ============================================================
% ====== 准确性优化 3：全向机器人运动学约束平滑 ==============
% ============================================================

function smoothed = smoothPathOmni(raw_path, max_seg_len, n_resample, max_accel)
    % 全向移动机器人路径平滑：
    %   1. 在过长的相邻段插入中间点（保证段长 <= max_seg_len）
    %   2. 用三次样条插值生成平滑曲线（全向机器人无最小转弯半径限制）
    %   3. 检查曲率突变，若加速度等效值超限则降低密度
    %
    % 注：全向机器人虽然可任意方向运动，但实际控制时仍需考虑速度连续性。
    %     这里通过限制段长 + 样条插值确保速度过渡平滑。

    if size(raw_path, 1) < 2
        smoothed = raw_path;
        return;
    end

    % --- Step 1: 段长约束，过长段插入中间点 ---
    densified = densifyPath(raw_path, max_seg_len);

    if size(densified, 1) < 3
        % 点数太少无法做样条，直接返回
        smoothed = densified;
        return;
    end

    % --- Step 2: 三次样条参数化平滑 ---
    % 用累积弧长作为参数（比简单等间隔参数更稳定）
    cumlen = [0; cumsum(sqrt(sum(diff(densified).^2, 2)))];
    if cumlen(end) < 1e-6
        smoothed = densified;
        return;
    end

    % 重采样参数
    t_query = linspace(0, cumlen(end), n_resample)';

    % 分别对 x, y 做三次样条
    try
        x_smooth = spline(cumlen, densified(:,1), t_query);
        y_smooth = spline(cumlen, densified(:,2), t_query);
    catch
        % 样条失败时退回到线性插值
        x_smooth = interp1(cumlen, densified(:,1), t_query, 'linear');
        y_smooth = interp1(cumlen, densified(:,2), t_query, 'linear');
    end

    smoothed = [x_smooth, y_smooth];

    % --- Step 3: 加速度等效约束检查 ---
    % 对全向机器人来说，加速度突变 = 二阶差分突变
    % 假设单位时间步长，二阶差分模 ≈ 等效加速度
    if size(smoothed, 1) >= 3
        d2 = diff(smoothed, 2);  % 二阶差分
        accel_norm = sqrt(sum(d2.^2, 2));
        if max(accel_norm) > max_accel
            % 用移动平均做二次平滑，抑制加速度突变
            window = 3;
            smoothed(:,1) = movmean(smoothed(:,1), window);
            smoothed(:,2) = movmean(smoothed(:,2), window);
        end
    end
end

function dense = densifyPath(path, max_seg_len)
    % 在路径段长超过 max_seg_len 时插入中间点
    if size(path, 1) < 2
        dense = path;
        return;
    end

    out = path(1, :);
    for i = 2:size(path, 1)
        seg_vec = path(i,:) - path(i-1,:);
        seg_len = norm(seg_vec);
        if seg_len > max_seg_len
            n_insert = ceil(seg_len / max_seg_len);
            for k = 1:n_insert
                t = k / n_insert;
                pt = path(i-1,:) + t * seg_vec;
                out(end+1, :) = pt; %#ok<AGROW>
            end
        else
            out(end+1, :) = path(i, :); %#ok<AGROW>
        end
    end
    dense = out;
end

% ============================================================
% ============== 实时性辅助函数（沿用上一版）==============
% ============================================================

function [img, newFut] = safeFetch(fut, Client, hasParallel, timeout)
    if ~hasParallel || isempty(fut)
        img = ImageReadTCP_One(Client, 'Center');
        newFut = [];
        return;
    end
    if ~isvalid(fut)
        img = ImageReadTCP_One(Client, 'Center');
        newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
        return;
    end

    % 有时限地等待 future 完成（这是之前 timeout 参数没有真正生效的地方）
    finished_in_time = wait(fut, 'finished', timeout);

    if finished_in_time && strcmp(fut.State, 'finished') && ~fut.Read
        % future 已完成且未读，正常取结果
        try
            img = fetchOutputs(fut);
        catch
            % fetchOutputs 出错（worker 内部异常），同步兜底
            img = ImageReadTCP_One(Client, 'Center');
        end
    else
        % 超时或状态异常：取消旧 future，同步读取当前帧保证不卡
        try
            cancel(fut);
        catch
        end
        img = ImageReadTCP_One(Client, 'Center');
    end

    % 立即派发下一帧（与本帧处理并行）
    try
        newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    catch
        newFut = [];
    end
end

function newFut = resetFuture(fut, Client, hasParallel)
    if ~hasParallel
        newFut = [];
        return;
    end
    if ~isempty(fut) && isvalid(fut)
        try
            if any(strcmp(fut.State, {'running', 'queued', 'pending', 'unavailable'}))
                cancel(fut);
            end
        catch
        end
    end
    try
        newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    catch
        newFut = [];
    end
end