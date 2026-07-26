%% 双层地图方案：永久静态层 + 动态观测层 + 可见性判断
%% 解决"遮挡导致静态障碍物误删"与"动态障碍物无法清除"的矛盾
%% 适用于：全向移动机器人 + Unity 鱼眼结构光

clear;

% ===== 初始化 =====
name = "Matlab";
Client = TCPInit('127.0.0.1', 55014, name);
load('Omni_Calib_Results_Unity.mat');
ocam_model = calib_data.ocam_model;

cvsyst_rot = 0;
camY = 0; camX = 0; camZ = 0;
lasY = 0.0; lasX = 0.0;
las_dist = 1850;

start_point = [0, 0];
goal_point  = [18000, 50];

robot_trace     = [];
safety_distance = 1600;

% ===== 实时性参数 =====
VIS_DECIMATE_GLOBAL   = 5;
ASYNC_TIMEOUT         = 0.2;
OBSTACLE_EXTRACT_FREQ = 5;
COLLISION_CHECK_FREQ  = 3; %多少帧后重新规划路线

% ===== 双层地图参数 =====
GRID_RESOLUTION = 150;
GRID_ORIGIN     = [-15000, -15000];
GRID_SIZE       = [400, 200];

% --- 动态观测层 ---
DYN_HIT_THRESHOLD = 1;      % 命中>=N次才显示（=1表示单帧即响应，最快）
DYN_DECAY_SECONDS = 2;    % 超过N秒未命中则清零（缩短，快速响应移动障碍物）
DECAY_INTERVAL    = 3;      % 每N帧执行一次衰减（加快衰减检查频率）

% --- 永久静态层 ---
STATIC_HIT_THRESHOLD  = 5;     % 命中>=N次才写入静态层
LASER_MAX_RANGE       = 8000;  % 激光最大量程 (mm)
STATIC_MISS_SECONDS   = 6.0;   % 可见范围内持续N秒未命中才删除
STATIC_DECAY_INTERVAL = 20;

% --- 路径规划 ---
LOCAL_PLAN_RADIUS    = 6000;
REPLAN_COOLDOWN      = 1;  %重新规划冷却时间
SAFETY_DIST_FALLBACK = [safety_distance, safety_distance*0.7, safety_distance*0.5];

% --- 运动学约束 ---
MAX_SEGMENT_LENGTH = 1500;
SMOOTH_RESAMPLE_N  = 30;
MAX_ACCELERATION   = 2000;

% ===== 初始化双层栅格地图 =====
% 动态层：快速衰减，响应移动障碍物
dyn_hit_count  = zeros(GRID_SIZE(1), GRID_SIZE(2), 'int16');
dyn_last_seen  = zeros(GRID_SIZE(1), GRID_SIZE(2), 'single');

% 静态层：永久保留，仅在可见+持续未命中时删除
static_map       = false(GRID_SIZE(1), GRID_SIZE(2));
static_hit_acc   = zeros(GRID_SIZE(1), GRID_SIZE(2), 'int16');
static_last_seen = zeros(GRID_SIZE(1), GRID_SIZE(2), 'single');

t_start = tic;

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

% --- Figure 2：静态层地图（蓝色，永久障碍物 + 轨迹 + 规划路径）---
figure;
h_static_map = scatter([], [], 4, [0.2 0.4 0.8], 'filled'); hold on;
h_trace_s    = plot(nan, nan, 'g-',  'LineWidth', 1.5);
h_path_s     = plot(nan, nan, 'm--', 'LineWidth', 1.0);
title('Static Layer Map (Permanent Obstacles)');
xlabel('X/mm'); ylabel('Y/mm'); grid on;
xlim([-10000, 20000]); ylim([-10000, 20000]);
legend({'Static obstacles','Robot path','Planned path'},'Location','best');

% --- Figure 3：动态层地图（橙色，实时移动障碍物 + 轨迹）---
figure;
h_dyn_map  = scatter([], [], 4, [1.0 0.4 0.0], 'filled'); hold on;
h_trace_d  = plot(nan, nan, 'g-',  'LineWidth', 1.5);
h_path_d   = plot(nan, nan, 'm--', 'LineWidth', 1.0);
title('Dynamic Layer Map (Moving Obstacles, Fast Decay)');
xlabel('X/mm'); ylabel('Y/mm'); grid on;
xlim([-10000, 20000]); ylim([-10000, 20000]);
legend({'Dynamic obstacles','Robot path','Planned path'},'Location','best');

% ===== 初始路径规划 =====
b = 1;
cvsyst_x = start_point(1);
cvsyst_y = start_point(2);

[x1_init, y1_init] = mapping(binaryImage, cvsyst_rot, 0, 0, ...
                              camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
if size(x1_init, 2) > 1; x1_init(:,1) = []; y1_init(:,1) = []; end
xy2_init = [x1_init(:)+cvsyst_x, y1_init(:)+cvsyst_y];
t_now = toc(t_start);

[dyn_hit_count, dyn_last_seen] = updateDynLayer(...
    dyn_hit_count, dyn_last_seen, xy2_init, GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, t_now);
[static_hit_acc, static_map, static_last_seen] = updateStaticLayer(...
    static_hit_acc, static_map, static_last_seen, xy2_init, ...
    GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, STATIC_HIT_THRESHOLD, t_now);

init_obstacles = safeObstacles(xy2_init);
fprintf('[INIT] 起点=(%.0f,%.0f) 终点=(%.0f,%.0f)\n', ...
        start_point(1),start_point(2),goal_point(1),goal_point(2));
fprintf('[INIT] 初始障碍物点数=%d，安全距离=%.0fmm\n', size(init_obstacles,1), safety_distance);
fprintf('[INIT] x范围:[%.0f,%.0f] y范围:[%.0f,%.0f]\n', ...
        min(init_obstacles(:,1)),max(init_obstacles(:,1)), ...
        min(init_obstacles(:,2)),max(init_obstacles(:,2)));

path = rrt_star_path_planning(start_point, goal_point, init_obstacles, safety_distance);
smoothed_path = smoothPathOmni(path, MAX_SEGMENT_LENGTH, SMOOTH_RESAMPLE_N, MAX_ACCELERATION);

figure;
scatter(init_obstacles(:,1), init_obstacles(:,2), 10, 'red', 'filled'); hold on;
plot(start_point(1),start_point(2),'go','MarkerSize',10,'LineWidth',2);
plot(goal_point(1), goal_point(2), 'bo','MarkerSize',10,'LineWidth',2);
plot(smoothed_path(:,1), smoothed_path(:,2), 'k-', 'LineWidth', 2);
axis equal; xlabel('X/mm'); ylabel('Y/mm'); title('Initial Path Planning'); hold off;

q = smoothed_path;
cvsyst_x = q(b,1); cvsyst_y = q(b,2);
current_position = q(b,:);
confirmed_obstacles = zeros(0, 2);

% ===== 启动异步预取 =====
hasParallel = ~isempty(ver('parallel'));
if hasParallel
    pool = gcp('nocreate');
    if isempty(pool); parpool('local', 1); end
    fut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    fprintf('[INFO] 异步预取已启用\n');
else
    fut = [];
    fprintf('[WARN] 无 Parallel Toolbox，回退同步模式\n');
end

frame_count = 0;
replan_cooldown_counter = 0;
t_fetch=0; t_laser=0; t_map=0; t_grid=0; t_coll=0; t_ctrl=0; t_vis=0;
tic_loop = tic;

% ===== 主循环 =====
while true
    try
        frame_count = frame_count + 1;
        t_now = toc(t_start);

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
        if size(x1, 2) > 1; x1(:,1) = []; y1(:,1) = []; end
        x2 = x1 + cvsyst_x;
        y2 = y1 + cvsyst_y;
        xy2 = [x2(:), y2(:)];
        t_map = t_map + toc(t0)*1000;

        % --- Step 4: 双层地图更新 ---
        t0 = tic;

        % 4a: 动态层更新（每帧）
        [dyn_hit_count, dyn_last_seen] = updateDynLayer(...
            dyn_hit_count, dyn_last_seen, xy2, ...
            GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, t_now);

        % 4b: 动态层衰减（定期，时间阈值与帧率无关）
        if mod(frame_count, DECAY_INTERVAL) == 0
            dyn_hit_count = decayDynLayer(...
                dyn_hit_count, dyn_last_seen, t_now, DYN_DECAY_SECONDS);
        end

        % 4c: 静态层更新（每帧，累计命中达阈值写入）
        [static_hit_acc, static_map, static_last_seen] = updateStaticLayer(...
            static_hit_acc, static_map, static_last_seen, xy2, ...
            GRID_ORIGIN, GRID_RESOLUTION, GRID_SIZE, STATIC_HIT_THRESHOLD, t_now);

        % 4d: 静态层可见性检查（定期）
        %     核心逻辑：只删除"在激光量程内且持续未命中"的栅格
        %     超出量程的保留（遮挡/超距，无法判断是否存在）
        if mod(frame_count, STATIC_DECAY_INTERVAL) == 0
            static_map = checkStaticVisibility(...
                static_map, static_last_seen, current_position, t_now, ...
                LASER_MAX_RANGE, STATIC_MISS_SECONDS, GRID_ORIGIN, GRID_RESOLUTION);
        end

        % 4e: 合并两层 → 提供给规划器（降频提取）
        if mod(frame_count, OBSTACLE_EXTRACT_FREQ) == 0 || isempty(confirmed_obstacles)
            confirmed_obstacles = mergeObstacleLayers(...
                static_map, dyn_hit_count, DYN_HIT_THRESHOLD, GRID_ORIGIN, GRID_RESOLUTION);
        end
        t_grid = t_grid + toc(t0)*1000;

        % --- Step 5: 碰撞检测 + 局部重规划 ---
        t0 = tic;
        replan_cooldown_counter = max(0, replan_cooldown_counter - 1);
        need_replan = false;
        if replan_cooldown_counter == 0 && ~isempty(confirmed_obstacles) && ...
           mod(frame_count, COLLISION_CHECK_FREQ) == 0
            need_replan = check_path_collision(smoothed_path, confirmed_obstacles, safety_distance);
        end

        if need_replan
            disp('路径冲突，启动局部重规划...');
            sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
            local_obs = safeObstacles(filterLocalObstacles(...
                confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS));
            new_path = tryReplan(current_position, sub_goal, local_obs, SAFETY_DIST_FALLBACK);
            if ~isempty(new_path)
                smoothed_path = smoothPathOmni(new_path, MAX_SEGMENT_LENGTH, ...
                                               SMOOTH_RESAMPLE_N, MAX_ACCELERATION);
                q = smoothed_path; b = 1;
                replan_cooldown_counter = REPLAN_COOLDOWN;
            else
                warning('[REPLAN] 全部失败，保持当前路径');
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
            cvsyst_x = q(b,1); cvsyst_y = q(b,2);
            b = b + 1;

            if norm(current_position - q(end,:)) < 300 && ...
               norm(current_position - goal_point) > 500
                disp('接近子目标，规划下一段...');
                sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
                local_obs = safeObstacles(filterLocalObstacles(...
                    confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS));
                new_path = tryReplan(current_position, sub_goal, local_obs, SAFETY_DIST_FALLBACK);
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
                fprintf('[NAV] 距终点%.0fmm，规划下一段...\n', dist_to_goal);
                sub_goal = computeSubGoal(current_position, goal_point, LOCAL_PLAN_RADIUS);
                local_obs = safeObstacles(filterLocalObstacles(...
                    confirmed_obstacles, current_position, LOCAL_PLAN_RADIUS));
                new_path = tryReplan(current_position, sub_goal, local_obs, SAFETY_DIST_FALLBACK);
                if ~isempty(new_path)
                    smoothed_path = smoothPathOmni(new_path, MAX_SEGMENT_LENGTH, ...
                                                   SMOOTH_RESAMPLE_N, MAX_ACCELERATION);
                    q = smoothed_path; b = 1;
                    replan_cooldown_counter = REPLAN_COOLDOWN;
                end
            else
                fprintf('[NAV] 已到达终点。\n');
            end
        end
        t_ctrl = t_ctrl + toc(t0)*1000;

        % --- Step 7: 可视化 ---
        t0 = tic;
        % 实时图（每帧）
        set(h1, 'CData', black_image);
        set(h2, 'CData', binaryImage);
        set(h3, 'XData', x1, 'YData', y1);
        set(h4, 'XData', x2, 'YData', y2);

        % 动态层 figure：每帧刷新（反映移动障碍物的实时变化）
        dyn_pts = extractLayerPoints(...
            dyn_hit_count >= int16(DYN_HIT_THRESHOLD), GRID_ORIGIN, GRID_RESOLUTION);
        if ~isempty(dyn_pts)
            set(h_dyn_map, 'XData', dyn_pts(:,1), 'YData', dyn_pts(:,2));
        else
            set(h_dyn_map, 'XData', [], 'YData', []);  % 障碍物消失时清空
        end
        set(h_trace_d, 'XData', robot_trace(:,1), 'YData', robot_trace(:,2));
        set(h_path_d,  'XData', smoothed_path(:,1), 'YData', smoothed_path(:,2));

        % 静态层 figure：降频刷新（静态障碍物变化慢，不需要每帧）
        if mod(frame_count, VIS_DECIMATE_GLOBAL) == 0
            static_pts = extractLayerPoints(static_map, GRID_ORIGIN, GRID_RESOLUTION);
            if ~isempty(static_pts)
                set(h_static_map, 'XData', static_pts(:,1), 'YData', static_pts(:,2));
            end
            set(h_trace_s, 'XData', robot_trace(:,1), 'YData', robot_trace(:,2));
            set(h_path_s,  'XData', smoothed_path(:,1), 'YData', smoothed_path(:,2));
        end

        drawnow limitrate;
        t_vis = t_vis + toc(t0)*1000;

        % --- 性能统计 ---
        if mod(frame_count, 5) == 0
            elapsed = toc(tic_loop);
            fps  = frame_count / elapsed;
            n    = frame_count;
            fprintf('[PERF] frame=%d FPS=%.1f | static=%d dyn=%d | b=%d/%d pos=(%.0f,%.0f)\n', ...
                    frame_count, fps, nnz(static_map), ...
                    nnz(dyn_hit_count>=int16(DYN_HIT_THRESHOLD)), ...
                    b, size(q,1), current_position(1), current_position(2));
            fprintf('  ms/帧: 取帧=%5.0f 激光=%5.0f mapping=%5.0f 栅格=%4.0f 碰撞=%4.0f 控制=%4.0f 可视=%4.0f\n', ...
                    t_fetch/n,t_laser/n,t_map/n,t_grid/n,t_coll/n,t_ctrl/n,t_vis/n);
        end

    catch ME
        warning('帧处理错误: %s', char(ME.message));
        fut = resetFuture(fut, Client, hasParallel);
        continue;
    end
end

% ===== 清理 =====
if hasParallel && ~isempty(fut) && isvalid(fut); cancel(fut); end


% ================================================================
% ==================== 双层地图核心函数 ==========================
% ================================================================

function [dyn_hit, dyn_seen] = updateDynLayer(dyn_hit, dyn_seen, xy_pts, ...
                                               origin, resolution, grid_size, timestamp)
    if isempty(xy_pts); return; end
    [ix, iy, valid] = worldToGrid(xy_pts, origin, resolution, grid_size);
    if ~any(valid); return; end
    cells   = unique([ix(valid), iy(valid)], 'rows');
    lin_idx = sub2ind([grid_size(1), grid_size(2)], cells(:,1), cells(:,2));
    dyn_hit(lin_idx)  = min(dyn_hit(lin_idx) + int16(1), int16(32000));
    dyn_seen(lin_idx) = single(timestamp);
end

function dyn_hit = decayDynLayer(dyn_hit, dyn_seen, current_time, decay_seconds)
    % 超过 decay_seconds 秒未命中 → 直接清零（消除移走的动态障碍物）
    stale = (dyn_hit > 0) & (single(current_time) - dyn_seen > single(decay_seconds));
    dyn_hit(stale) = int16(0);
end

function [static_acc, static_map, static_seen] = updateStaticLayer(...
        static_acc, static_map, static_seen, xy_pts, ...
        origin, resolution, grid_size, threshold, timestamp)
    if isempty(xy_pts); return; end
    [ix, iy, valid] = worldToGrid(xy_pts, origin, resolution, grid_size);
    if ~any(valid); return; end
    cells   = unique([ix(valid), iy(valid)], 'rows');
    lin_idx = sub2ind([grid_size(1), grid_size(2)], cells(:,1), cells(:,2));
    % 累计命中，达阈值写入静态地图
    static_acc(lin_idx) = min(static_acc(lin_idx) + int16(1), int16(32000));
    static_map = static_map | (static_acc >= int16(threshold));
    % 更新被命中栅格的时间戳（用于可见性判断）
    static_seen(lin_idx) = single(timestamp);
end

function static_map = checkStaticVisibility(...
        static_map, static_seen, robot_pos, current_time, ...
        laser_range, miss_seconds, origin, resolution)
    % 可见性判断：对静态地图中每个障碍物栅格：
    %   在激光量程内（机器人应该能看到）+ 持续未命中 → 真正消失 → 删除
    %   超出量程 → 保留（被遮挡或超距，无法判断是否存在）
    [ix, iy] = find(static_map);
    if isempty(ix); return; end

    cx = origin(1) + (double(ix) - 0.5) * resolution;
    cy = origin(2) + (double(iy) - 0.5) * resolution;
    dist = sqrt((cx - robot_pos(1)).^2 + (cy - robot_pos(2)).^2);

    lin_idx = sub2ind(size(static_map), ix, iy);
    time_since_seen = single(current_time) - static_seen(lin_idx);

    % 只删除"在量程内但持续未命中"的栅格
    remove = (dist <= laser_range) & (time_since_seen > single(miss_seconds));
    if any(remove)
        fprintf('[MAP] 删除 %d 个消失的静态障碍物（可见范围内持续未命中）\n', sum(remove));
        static_map(lin_idx(remove)) = false;
    end
end

function obstacles = mergeObstacleLayers(static_map, dyn_hit, dyn_threshold, origin, resolution)
    % 规划器用障碍物 = 静态层 OR 动态层（取并集）
    merged    = static_map | (dyn_hit >= int16(dyn_threshold));
    obstacles = extractLayerPoints(merged, origin, resolution);
end

function pts = extractLayerPoints(layer_mask, origin, resolution)
    [ix, iy] = find(layer_mask);
    if isempty(ix); pts = zeros(0,2); return; end
    pts = [origin(1)+(double(ix(:))-0.5)*resolution, ...
           origin(2)+(double(iy(:))-0.5)*resolution];
end

function [ix, iy, valid] = worldToGrid(xy_pts, origin, resolution, grid_size)
    ix    = floor((xy_pts(:,1) - origin(1)) / resolution) + 1;
    iy    = floor((xy_pts(:,2) - origin(2)) / resolution) + 1;
    valid = ix >= 1 & ix <= grid_size(1) & iy >= 1 & iy <= grid_size(2);
end


% ================================================================
% ==================== 路径规划辅助函数 ==========================
% ================================================================

function new_path = tryReplan(current_pos, sub_goal, local_obs, safety_dist_list)
    new_path = [];
    for sd = safety_dist_list
        try
            new_path = rrt_star_path_planning(current_pos, sub_goal, local_obs, sd);
            fprintf('[REPLAN] 成功，安全距离=%.0fmm\n', sd);
            return;
        catch
            fprintf('[REPLAN] 安全距离=%.0fmm 失败\n', sd);
        end
    end
end

function sub_goal = computeSubGoal(current_pos, real_goal, local_radius)
    delta = real_goal - current_pos;
    dist  = norm(delta);
    if dist <= local_radius; sub_goal = real_goal;
    else; sub_goal = current_pos + delta/dist * local_radius;
    end
end

function local_obs = filterLocalObstacles(all_obs, center, radius)
    if isempty(all_obs); local_obs = []; return; end
    dx = all_obs(:,1) - center(1);
    dy = all_obs(:,2) - center(2);
    local_obs = all_obs(dx.^2 + dy.^2 <= radius^2, :);
end

function obs = safeObstacles(obs)
    if isempty(obs); obs = zeros(0,2); return; end
    if isvector(obs); obs = obs(:); end
    if size(obs,2) ~= 2; obs = zeros(0,2); end
end


% ================================================================
% ==================== 路径平滑函数 ==============================
% ================================================================

function smoothed = smoothPathOmni(raw_path, max_seg_len, n_resample, max_accel)
    if size(raw_path,1) < 2; smoothed = raw_path; return; end
    densified = densifyPath(raw_path, max_seg_len);
    if size(densified,1) < 3; smoothed = densified; return; end
    cumlen = [0; cumsum(sqrt(sum(diff(densified).^2, 2)))];
    if cumlen(end) < 1e-6; smoothed = densified; return; end
    t_query = linspace(0, cumlen(end), n_resample)';
    try
        x_s = spline(cumlen, densified(:,1), t_query);
        y_s = spline(cumlen, densified(:,2), t_query);
    catch
        x_s = interp1(cumlen, densified(:,1), t_query, 'linear');
        y_s = interp1(cumlen, densified(:,2), t_query, 'linear');
    end
    smoothed = [x_s, y_s];
    if size(smoothed,1) >= 3
        d2 = diff(smoothed, 2);
        if max(sqrt(sum(d2.^2,2))) > max_accel
            smoothed(:,1) = movmean(smoothed(:,1), 3);
            smoothed(:,2) = movmean(smoothed(:,2), 3);
        end
    end
end

function dense = densifyPath(path, max_seg_len)
    if size(path,1) < 2; dense = path; return; end
    out = path(1,:);
    for i = 2:size(path,1)
        seg = path(i,:) - path(i-1,:);
        len = norm(seg);
        if len > max_seg_len
            n = ceil(len / max_seg_len);
            for k = 1:n
                out(end+1,:) = path(i-1,:) + (k/n)*seg; %#ok<AGROW>
            end
        else
            out(end+1,:) = path(i,:); %#ok<AGROW>
        end
    end
    dense = out;
end


% ================================================================
% ==================== 异步取帧函数 ==============================
% ================================================================

function [img, newFut] = safeFetch(fut, Client, hasParallel, timeout)
    if ~hasParallel || isempty(fut)
        img = ImageReadTCP_One(Client, 'Center'); newFut = []; return;
    end
    if ~isvalid(fut)
        img = ImageReadTCP_One(Client, 'Center');
        newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center'); return;
    end
    ok = wait(fut, 'finished', timeout);
    if ok && strcmp(fut.State,'finished') && ~fut.Read
        try; img = fetchOutputs(fut);
        catch; img = ImageReadTCP_One(Client, 'Center'); end
    else
        try; cancel(fut); catch; end
        img = ImageReadTCP_One(Client, 'Center');
    end
    try; newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    catch; newFut = []; end
end

function newFut = resetFuture(fut, Client, hasParallel)
    if ~hasParallel; newFut = []; return; end
    if ~isempty(fut) && isvalid(fut)
        try
            if any(strcmp(fut.State,{'running','queued','pending','unavailable'}))
                cancel(fut);
            end
        catch; end
    end
    try; newFut = parfeval(@ImageReadTCP_One, 1, Client, 'Center');
    catch; newFut = []; end
end