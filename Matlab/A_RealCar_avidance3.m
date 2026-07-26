%% 激光建图 + HC-08 小车 联合避障测试(前进/后退/转向版)
%  Real fisheye laser mapping + HC-08 car -> forward/back/turn avoidance.
%
%  工作流程 / Pipeline:
%    snapshot -> extractRealRedLaser -> mapping -> 障碍判断 -> 发送 A|n|$ 指令
%
%  运动模型 / Motion model (来自 app_control.ino):
%    指令 "设定即保持":每帧发一条方向指令,小车保持该方向到下一帧。
%    A|<state>|$  运动;  C|<speed>|$  调速;  A|8|$  停止。
%    state: 2=前进 6=后退 9=左转 10=右转 8=停止
%    不再使用左右平移和斜向运动；转向会改变车头/相机/激光朝向。
%
%  停止方式 / How to stop:
%    关闭图窗即可干净停车;若按 Ctrl+C,请立刻在命令行输入  car.stop()
%
%  首次测试强烈建议把小车架空(车轮离地)先观察决策是否正确,再落地。
%
%  依赖 / Dependencies (须在路径上):
%    connectMiniAutoHC08.m, mapping.m, extractRealRedLaser.m, 标定文件 .mat

clear;
clc;

thisDir = fileparts(mfilename("fullpath"));
addpath(thisDir);

%% ===== 相机 / 标定 / 激光提取参数(沿用真实小车配置)=====
CAMERA_INDEX       = 2;
CAMERA_RESOLUTION  = '1920x1080';
CAMERA_BRIGHTNESS  = -30;
CALIBRATION_FILE   = 'Omni_Calib_Results_Real_Car.mat';   % 真实小车摄像头标定

camX = 2;   camY = 0;   camZ = -2;
lasX = -0.75;  lasY = 0.085;  las_dist = 175;
CVsyst_rot = 0;  CVsyst_x = 0;  CVsyst_y = 0;

MIN_RED_EXCESS             = 18;
MIN_RED_VALUE              = 90;
MIN_HSV_SATURATION         = 0.06;
MIN_HSV_VALUE              = 0.80;
MIN_COMPONENT_AREA         = 45;
MAX_COMPONENT_AREA         = 5000;
MIN_COMPONENT_ECCENTRICITY = 0.60;
MIN_COMPONENT_MAJOR_AXIS   = 25;

%% ===== 避障决策参数 =====
%  激光地图坐标系: map_y = 前方距离(mm), map_x = 横向偏移(mm,负=左 正=右)
%  以下所有距离/宽度阈值单位均为 mm。
av.senseRange        = 600;   % 只关注前方 0~600mm 的障碍点
av.corridorHalfWidth = 150;   % 正前方“走廊”半宽(mm),左右各 150mm,总宽 300mm
av.slowDist          = 400;   % 进入此距离:斜向前进避让(保持前进)
av.stopDist          = 200;   % 进入此距离:纯侧移让开走廊(必要时后退)

DRIVE_SPEED          = 20;    % 巡航速度(0~100,慢一点更安全;斜向时固件自动 /√2)
FLIP_STEERING        = false; % 若小车避让方向反了,改成 true

%% ===== 第一阶段:模糊避障速度/方向参数(不改小车固件) =====
% 输入:最近障碍距离、距离变化率、左右空旷程度
% 输出:C|speed|$ 与 A|state|$。这不是轮级 PID,但能让真实避障更平滑。
fz.minSpeed          = 14;    % 有危险时仍可侧移/后退的最低速度
fz.backupSpeed       = 18;    % 后退速度上限
fz.emergencyDist     = 110;   % 极近距离触发后退脱困
fz.rateRiskScale     = 320;   % 障碍快速接近时的风险归一化尺度(mm/s)
fz.speedRateLimit    = 45;    % 速度指令最大变化率(速度单位/s)
fz.speedCmdDeadband  = 2;     % 速度变化小于此值时不重复发 C 指令
fz.stateHoldTime     = 0.45;  % 方向最短保持时间(s),减少左右转抖动
fz.turnSpeed         = 16;    % 转向速度
fz.turnDeadbandDeg   = 18;    % 目标方向在此角度内视为正前方,直接前进
fz.reverseHeadingDeg = 155;   % 目标在车后超过此角度时改为后退
fz.avoidCommitTime   = 1.20;  % 选定避让方向后的最短承诺时间(s)
fz.turnBeforeCreepTime = 0.90;% 转向避让后允许试探性前进的时间(s)
fz.stuckTime         = 0.90;  % 贴近障碍持续此时间仍脱不了困 -> 触发后退脱困(s)
fz.backupCommitTime  = 0.80;  % 一旦决定后退,至少后退这么久,避免与转向抖动(s)

%% ===== 局部路径规划参数(ARealTimeTest8_2 的 RRT* -> 8方向可执行路径思想) =====
% 坐标系: x<0 左, x>0 右, y>0 前。每帧以小车当前位置 [0,0] 规划到前方局部目标。
pp.enabled           = true;
pp.goal              = [0, 650];   % rolling local target distance, mm
pp.globalGoal        = [0, 2000];  % fixed final target in world coordinates, mm
pp.localGoalDist     = norm(pp.goal);
pp.replanInterval    = 0.60;       % replan interval, seconds
pp.safetyDistList    = [210 170 130];  % planning safety distance, mm
pp.execSafetyExtra   = 25;             % extra validation margin after path conversion
pp.execStepLen       = 80;         % max executable path segment length, mm
pp.maxObstaclePoints = 220;        % obstacle point limit for RRT
pp.lookaheadDist     = 260;        % path tracking lookahead distance, mm
pp.maxReplanHold     = 2.00;       % max old-path hold time while turning, seconds
pp.goalReachDist     = 80;         % final target reach threshold, mm
pp.xLimit            = [-650 650];
pp.yLimit            = [-120 1050];
pp.rrtMaxIter        = 850;
pp.rrtStep           = 90;
pp.rrtGoalTolerance  = 100;
pp.rrtNeighborRadius = 350;
lastRawPath          = zeros(0, 2);
lastExecPath         = zeros(0, 2);
plannerStatus        = "init";
plannerSafetyDist    = NaN;

%% ===== 开环当前位置估计(用于让重规划从当前位置开始) =====
% 速度单位到 mm/s 的比例需要实车标定；只用于规划起点/显示，不是精确定位。
pose.wheelDiameterMm = 60;                         % wheel diameter (mm)
pose.wheelCircumferenceMm = pi * pose.wheelDiameterMm;
pose.loadedWheelRpm = 64;                           % loaded output speed after gearbox (rpm)
pose.maxCommandSpeed = 100;                         % C|speed|$ command full scale
pose.trackWidthMm = 164;                            % approximate left-right wheel track / car width (mm)
pose.maxLinearMmPerSec = pose.loadedWheelRpm * pose.wheelCircumferenceMm / 60;
pose.speedUnitToMmPerSec = pose.maxLinearMmPerSec / pose.maxCommandSpeed;
pose.maxTurnDegPerSec = rad2deg(pose.maxLinearMmPerSec / (pose.trackWidthMm / 2));
pose.turnDegPerSpeedUnitSec = pose.maxTurnDegPerSec / pose.maxCommandSpeed;
pose.headingRad = 0;
currentPos = [0, 0];

%  小车运动 state 编码(对应固件 Rockerandgravity_Task)
S.fwd      = 2;
S.back     = 6;
S.rotLeft  = 9;
S.rotRight = 10;
S.stop     = 8;
lastMotionState = S.stop;
lastMotionSpeed = 0;

%% ===== 载入标定 =====
calibration = load(CALIBRATION_FILE);
ocam_model  = calibration.calib_data.ocam_model;

%% ===== 打开相机 =====
available_cameras = webcamlist;
if CAMERA_INDEX > numel(available_cameras)
    error('webcam(%d) 不可用。检测到的相机: %s', ...
        CAMERA_INDEX, strjoin(string(available_cameras), ', '));
end
camera_device = webcam(CAMERA_INDEX);
try
    camera_device.Resolution = CAMERA_RESOLUTION;
catch ME
    warning('无法设置分辨率 %s: %s', CAMERA_RESOLUTION, ME.message);
end
try
    camera_device.Brightness = CAMERA_BRIGHTNESS;
catch ME
    warning('无法设置亮度 %.1f: %s', CAMERA_BRIGHTNESS, ME.message);
end
% camera_device.Exposure = -7;   % 激光过曝发白时可打开降低曝光

%% ===== 连接小车 =====
car = connectMiniAutoHC08();
cleanup = onCleanup(@() safeShutdown(car));   % 工作区清除时自动停车+断开

% 取得可写入 FFE1 的特征句柄(唯一适配点,见文件末尾 getTxCharacteristic)
txChar = getTxCharacteristic(car);

fuzzyState = initFuzzyAvoidanceState(DRIVE_SPEED, fz);
currentSpeed = DRIVE_SPEED;
lastSentSpeed = round(currentSpeed);

sendCmd(txChar, sprintf('C|%d|$', lastSentSpeed));       % 设速度
sendCmd(txChar, sprintf('A|%d|$', S.stop));              % 先停住

fprintf('[INIT] 相机 webcam(%d)=%s  分辨率=%s\n', ...
    CAMERA_INDEX, string(available_cameras{CAMERA_INDEX}), camera_device.Resolution);
fprintf('[INIT] 标定=%s  激光 las_dist=%.1f mm\n', CALIBRATION_FILE, las_dist);
fprintf('[INIT] 避障: stop<%.0f  slow<%.0f  走廊半宽=%.0f  速度=%d  全向无自转\n', ...
    av.stopDist, av.slowDist, av.corridorHalfWidth, DRIVE_SPEED);
fprintf('[INIT] 模糊避障: minSpeed=%d backupSpeed=%d emergency<%.0f stuck>%.2fs\n', ...
    fz.minSpeed, fz.backupSpeed, fz.emergencyDist, fz.stuckTime);

input('小车已连接。建议先架空车轮。按 Enter 开始避障测试,或 Ctrl+C 取消。', 's');

%% ===== 首帧与界面 =====
first_image = snapshot(camera_device);
first_mask  = false(size(first_image, 1), size(first_image, 2));

fig = figure('Name', 'Laser Mapping + Omnidirectional Avoidance', 'NumberTitle', 'off');

ax_image = subplot(1,3,1);
h_image  = imshow(first_image, 'Parent', ax_image);
title(ax_image, 'Real Fisheye Image');

ax_mask = subplot(1,3,2);
h_mask  = imshow(first_mask, 'Parent', ax_mask);
title(ax_mask, 'Extracted Laser');

ax_map = subplot(1,3,3);
h_map  = scatter(ax_map, nan, nan, 8, 'filled');
hold(ax_map, 'on');
plot(ax_map, CVsyst_x, CVsyst_y, 'r+', 'MarkerSize', 10, 'LineWidth', 1.5);
% 走廊边界(竖虚线)与距离阈值(横线)便于直观判断
plot(ax_map, [-av.corridorHalfWidth -av.corridorHalfWidth], [-100 700], 'k--');
plot(ax_map, [ av.corridorHalfWidth  av.corridorHalfWidth], [-100 700], 'k--');
plot(ax_map, [-300 300], [av.stopDist av.stopDist], 'r-',  'LineWidth', 1.0);
plot(ax_map, [-300 300], [av.slowDist av.slowDist], 'Color', [1 0.5 0]);
h_raw_path = plot(ax_map, nan, nan, 'c--', 'LineWidth', 1.1);
h_exec_path = plot(ax_map, nan, nan, 'm-', 'LineWidth', 2.0);
h_path_target = plot(ax_map, nan, nan, 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);
hold(ax_map, 'off');
axis(ax_map, 'equal'); grid(ax_map, 'on');
xlim(ax_map, [-450, 450]); ylim(ax_map, [-100, 900]);
xlabel(ax_map, 'X / mm'); ylabel(ax_map, 'Y / mm');
title(ax_map, 'Laser Mapping');

fig_plan = figure('Name', 'Local RRT* Turn-Drive Path Planning', 'NumberTitle', 'off');
ax_plan = axes(fig_plan);
h_plan_obs = scatter(ax_plan, nan, nan, 12, 'red', 'filled'); hold(ax_plan, 'on');
h_plan_raw = plot(ax_plan, nan, nan, 'c--', 'LineWidth', 1.2);
h_plan_exec = plot(ax_plan, nan, nan, 'm-', 'LineWidth', 2.0);
h_plan_target = plot(ax_plan, nan, nan, 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
h_plan_car = plot(ax_plan, 0, 0, 'gs', 'MarkerSize', 9, 'MarkerFaceColor', 'g', 'LineWidth', 1.2);
h_plan_heading = quiver(ax_plan, 0, 0, 0, 120, 0, 'Color', [0 0.55 0], 'LineWidth', 2.0, 'MaxHeadSize', 1.2);
h_plan_goal = plot(ax_plan, nan, nan, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
h_plan_info = text(ax_plan, pp.xLimit(1) + 20, pp.yLimit(2) - 35, '', ...
    'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', 'w', 'Margin', 4, 'EdgeColor', [0.7 0.7 0.7]);
axis(ax_plan, 'equal'); grid(ax_plan, 'on');
xlim(ax_plan, pp.xLimit); ylim(ax_plan, pp.yLimit);
xlabel(ax_plan, 'X / mm'); ylabel(ax_plan, 'Y / mm');
title(ax_plan, 'World path planning');
legend(ax_plan, {'Obstacles','Raw RRT*','Executable path','Next target','Car position','Car heading','Plan goal'}, 'Location', 'best');
hold(ax_plan, 'off');

frame_count = 0;
valid_frame_count = 0;
loop_clock = tic;
frame_clock = tic;
plan_clock = tic;

%% ===== 主循环 =====
while isvalid(fig)
    try
        frame_count = frame_count + 1;
        dt = max(toc(frame_clock), 1e-3);
        frame_clock = tic;
        [currentPos, pose.headingRad] = estimatePoseFromState( ...
            currentPos, pose.headingRad, lastMotionState, lastMotionSpeed, dt, ...
            pose.speedUnitToMmPerSec, pose.turnDegPerSpeedUnitSec, S);

        image_rgb = snapshot(camera_device);

        laser_mask = extractRealRedLaser(image_rgb, MIN_RED_EXCESS, ...
            MIN_RED_VALUE, MIN_HSV_SATURATION, MIN_HSV_VALUE, ...
            MIN_COMPONENT_AREA, MAX_COMPONENT_AREA, ...
            MIN_COMPONENT_ECCENTRICITY, MIN_COMPONENT_MAJOR_AXIS);

        [map_x, map_y] = mapping(laser_mask, CVsyst_rot, CVsyst_y, CVsyst_x, ...
            camY, camX, camZ, lasY, lasX, las_dist, ocam_model);

        % mapping() 第一点是 CV 原点,去掉
        if numel(map_x) > 1
            map_x = map_x(2:end);  map_y = map_y(2:end);
        else
            map_x = [];  map_y = [];
        end
        ok = isfinite(map_x) & isfinite(map_y);
        map_x = map_x(ok);  map_y = map_y(ok);

        % ---- 局部路径规划: RRT* 原始路径 -> 麦轮 8方向可执行路径 ----
        rawPath = lastRawPath;
        execPath = lastExecPath;
        localObstacles = [map_x(:), map_y(:)];
        worldObstacles = localToWorldPoints(localObstacles, currentPos, pose.headingRad);
        worldGoal = chooseRollingWorldGoal(currentPos, pp.globalGoal, pp.localGoalDist);
        isTurningCmd = lastMotionState == S.rotLeft || lastMotionState == S.rotRight;
        replanDue = toc(plan_clock) >= pp.replanInterval && ~isTurningCmd;
        replanForced = toc(plan_clock) >= pp.maxReplanHold;
        if pp.enabled && (isempty(lastExecPath) || replanDue || replanForced)
            [newRawPath, newExecPath, plannerStatus, newSafetyDist] = planLocalMecanumPath( ...
                worldObstacles, currentPos, worldGoal, pp);
            if ~isempty(newExecPath)
                lastRawPath = newRawPath;
                lastExecPath = newExecPath;
                plannerSafetyDist = newSafetyDist;
            else
                if isempty(lastExecPath)
                    lastRawPath = zeros(0, 2);
                    lastExecPath = zeros(0, 2);
                end
                plannerSafetyDist = newSafetyDist;
            end
            rawPath = lastRawPath;
            execPath = lastExecPath;
            plan_clock = tic;
        end
        rawPathLocal = pathToLocal(rawPath, currentPos, pose.headingRad);
        execPathLocal = pathToLocal(execPath, currentPos, pose.headingRad);
        pathTarget = choosePathTarget(execPath, currentPos, pose.headingRad, pp.lookaheadDist);
        pathTargetWorld = localToWorldPoints(pathTarget, currentPos, pose.headingRad);
        distToPlanGoal = norm(pp.globalGoal - currentPos);
        goalReached = distToPlanGoal <= pp.goalReachDist;

        % ---- 模糊决策(全向,零自转):方向 + 自适应速度 ----
        [action, state, targetSpeed, dbg, fuzzyState] = decidePlannedFuzzyOmniAvoidance( ...
            map_x, map_y, pathTarget, av, fz, S, FLIP_STEERING, fuzzyState, dt);
        dbg.planStatus = plannerStatus;
        dbg.planSafetyDist = plannerSafetyDist;

        currentSpeed = rampToward(currentSpeed, targetSpeed, fz.speedRateLimit * dt);
        cmdSpeed = round(currentSpeed);
        if abs(cmdSpeed - lastSentSpeed) >= fz.speedCmdDeadband
            sendCmd(txChar, sprintf('C|%d|$', cmdSpeed));
            lastSentSpeed = cmdSpeed;
        end

        % ---- 执行:发一条方向指令,小车保持到下一帧 ----
        sendCmd(txChar, sprintf('A|%d|$', state));
        lastMotionState = state;
        lastMotionSpeed = cmdSpeed;

        % ---- 显示 ----
        set(h_image, 'CData', image_rgb);
        set(h_mask,  'CData', laser_mask);
        set(h_map,   'XData', map_x, 'YData', map_y);
        setPathLine(h_raw_path, rawPathLocal);
        setPathLine(h_exec_path, execPathLocal);
        setPointMarker(h_path_target, pathTarget);
        setPathLine(h_plan_raw, rawPath);
        setPathLine(h_plan_exec, execPath);
        setPointMarker(h_plan_target, pathTargetWorld);
        setPointMarker(h_plan_car, currentPos);
        setHeadingArrow(h_plan_heading, currentPos, pose.headingRad, 120);
        setPointMarker(h_plan_goal, pp.globalGoal);
        setPoseInfoText(h_plan_info, currentPos, pose.headingRad, distToPlanGoal, goalReached, pp.goalReachDist);
        setScatterPoints(h_plan_obs, worldObstacles);
        setPlanWorldView(ax_plan, h_plan_info, currentPos, pp.globalGoal, pp);

        valid_frame_count = valid_frame_count + 1;
        fps = valid_frame_count / max(toc(loop_clock), eps);
        title(ax_map, sprintf('%s | plan=%s sd=%.0f | nearest=%.0fmm | speed=%d | %.1f FPS', ...
            action, char(dbg.planStatus), dbg.planSafetyDist, dbg.nearest, cmdSpeed, fps), 'Interpreter', 'none');
        if isgraphics(ax_plan)
            title(ax_plan, sprintf('World path planning | plan=%s safety=%.0fmm | distGoal=%.0fmm | reached=%d', ...
                char(dbg.planStatus), dbg.planSafetyDist, distToPlanGoal, goalReached), 'Interpreter', 'none');
        end

        if mod(frame_count, 10) == 0
            fprintf('[FRAME %d] action=%s plan=%s sd=%.0f state=%d speed=%d nearest=%.0f L=%.0f R=%.0f risk=%.2f rate=%.2f stuck=%.2f pix=%d\n', ...
                frame_count, action, char(dbg.planStatus), dbg.planSafetyDist, state, cmdSpeed, dbg.nearest, dbg.leftClear, ...
                dbg.rightClear, dbg.hazard, dbg.rateRisk, fuzzyState.stuckTimer, nnz(laser_mask));
        end

        drawnow limitrate;
    catch ME
        warning('[FRAME %d] %s', frame_count, ME.message);
        try, sendCmd(txChar, sprintf('A|%d|$', S.stop)); catch, end   % 出错立即停车
        pause(0.1);
    end
end

%% ===== 收尾 =====
try, sendCmd(txChar, sprintf('A|%d|$', S.stop)); catch, end
try, car.stop(); catch, end
clear camera_device;
fprintf('[STOP] 联合避障测试结束。\n');

%% ================= 局部函数 =================

function st = initFuzzyAvoidanceState(cruiseSpeed, fz)
%INITFUZZYAVOIDANCESTATE 保存模糊避障的历史量与方向保持状态。
    st.cruiseSpeed = cruiseSpeed;
    st.lastNearest = inf;
    st.lastState = 8;
    st.stateAge = inf;
    st.minSpeed = fz.minSpeed;
    st.avoidSide = 0;
    st.avoidAge = inf;
    st.stuckTimer  = 0;   % 在 stopDist 内且没远离的累计时间
    st.backupTimer = 0;   % 剩余的承诺后退时间
end

function [rawPath, execPath, status, usedSafetyDist] = planLocalMecanumPath(obstacles, start, goal, pp)
%PLANLOCALMECANUMPATH Plan an 8-direction executable path from current position.
    status = "failed";
    usedSafetyDist = NaN;
    rawPath = zeros(0, 2);
    execPath = zeros(0, 2);

    start = double(start(:).');
    goal = double(goal(:).');
    obstacles = safeObstacles(obstacles);
    obstacles = filterPlannerObstacles(obstacles, pp, start);

    for safetyDist = pp.safetyDistList
        try
            rawPath = rrtStarLocalPath(start, goal, obstacles, safetyDist, pp);
            execPath = buildMecanumExecutablePath(rawPath, pp.execStepLen);
            execSafetyDist = safetyDist + pp.execSafetyExtra;
            if ~isempty(execPath) && size(execPath, 1) >= 2 && ...
                    ~checkPathCollisionLocal(execPath, obstacles, execSafetyDist)
                usedSafetyDist = execSafetyDist;
                if size(rawPath, 1) == 2
                    status = "direct_clear";
                else
                    status = "rrt";
                end
                return;
            end
        catch
        end
    end

    rawPath = zeros(0, 2);
    execPath = zeros(0, 2);
    status = "failed";
end

function obstacles = filterPlannerObstacles(obstacles, pp, center)
%FILTERPLANNEROBSTACLES Keep obstacle points inside the local planning window.
    if isempty(obstacles)
        obstacles = zeros(0, 2);
        return;
    end

    rel = obstacles - center;
    ok = isfinite(obstacles(:,1)) & isfinite(obstacles(:,2)) & ...
        rel(:,1) >= pp.xLimit(1) & rel(:,1) <= pp.xLimit(2) & ...
        rel(:,2) >= pp.yLimit(1) & rel(:,2) <= pp.yLimit(2);
    obstacles = obstacles(ok, :);

    if size(obstacles, 1) > pp.maxObstaclePoints
        idx = round(linspace(1, size(obstacles, 1), pp.maxObstaclePoints));
        obstacles = obstacles(idx, :);
    end
end

function path = rrtStarLocalPath(start, goal, obstacles, safetyDist, pp)
%RRTSTARLOCALPATH 适合真实小车局部视野的小尺度 RRT*。
    obstacles = safeObstacles(obstacles);
    xLimit = start(1) + pp.xLimit;
    yLimit = start(2) + pp.yLimit;

    % 直线可行时仍返回直线,但调用方还会检查 8方向执行路径。
    if ~checkSegmentCollisionLocal(start, goal, obstacles, safetyDist)
        path = [start; goal];
        return;
    end

    tree = start;
    parent = -1;
    cost = 0;
    found = false;

    for iter = 1:pp.rrtMaxIter
        if rand < 0.25
            sample = goal;
        else
            sample = [xLimit(1), yLimit(1)] + rand(1,2) .* ...
                [diff(xLimit), diff(yLimit)];
        end

        [nearestIdx, nearestNode] = findNearestLocal(tree, sample);
        direction = sample - nearestNode;
        d = norm(direction);
        if d < 1e-6
            continue;
        end

        newNode = nearestNode + pp.rrtStep * direction / d;
        newNode(1) = clampValue(newNode(1), xLimit(1), xLimit(2));
        newNode(2) = clampValue(newNode(2), yLimit(1), yLimit(2));

        if checkSegmentCollisionLocal(nearestNode, newNode, obstacles, safetyDist)
            continue;
        end

        neighbors = findNeighborsLocal(tree, newNode, pp.rrtNeighborRadius);
        minCost = cost(nearestIdx) + norm(nearestNode - newNode);
        bestParent = nearestIdx;

        for j = 1:numel(neighbors)
            n = neighbors(j);
            candidateCost = cost(n) + norm(tree(n,:) - newNode);
            if candidateCost < minCost && ...
                    ~checkSegmentCollisionLocal(tree(n,:), newNode, obstacles, safetyDist)
                minCost = candidateCost;
                bestParent = n;
            end
        end

        tree = [tree; newNode]; %#ok<AGROW>
        parent = [parent; bestParent]; %#ok<AGROW>
        cost = [cost; minCost]; %#ok<AGROW>
        newIdx = size(tree, 1);

        for j = 1:numel(neighbors)
            n = neighbors(j);
            newCost = cost(newIdx) + norm(tree(n,:) - newNode);
            if newCost < cost(n) && ...
                    ~checkSegmentCollisionLocal(newNode, tree(n,:), obstacles, safetyDist)
                parent(n) = newIdx;
                cost(n) = newCost;
            end
        end

        if norm(newNode - goal) <= pp.rrtGoalTolerance && ...
                ~checkSegmentCollisionLocal(newNode, goal, obstacles, safetyDist)
            tree = [tree; goal]; %#ok<AGROW>
            parent = [parent; newIdx]; %#ok<AGROW>
            found = true;
            break;
        end
    end

    if ~found
        error('local RRT path not found');
    end

    path = backtrackLocalPath(tree, parent);
    path = pruneLocalPath(path, obstacles, safetyDist);
end

function path = backtrackLocalPath(tree, parent)
    path = [];
    idx = size(tree, 1);
    while idx > 0
        path = [tree(idx,:); path]; %#ok<AGROW>
        idx = parent(idx);
    end
end

function pruned = pruneLocalPath(path, obstacles, safetyDist)
    if isempty(path) || size(path, 1) <= 2
        pruned = path;
        return;
    end

    pruned = path(1,:);
    i = 1;
    while i < size(path, 1)
        j = size(path, 1);
        while j > i + 1
            if ~checkSegmentCollisionLocal(path(i,:), path(j,:), obstacles, safetyDist)
                break;
            end
            j = j - 1;
        end
        pruned = [pruned; path(j,:)]; %#ok<AGROW>
        i = j;
    end
end

function [idx, node] = findNearestLocal(tree, sample)
    [~, idx] = min(sum((tree - sample).^2, 2));
    node = tree(idx, :);
end

function neighbors = findNeighborsLocal(tree, node, radius)
    d = sqrt(sum((tree - node).^2, 2));
    neighbors = find(d <= radius);
end

function collision = checkSegmentCollisionLocal(p1, p2, obstacles, safetyDist)
    collision = false;
    if isempty(obstacles)
        return;
    end

    segLen = norm(p2 - p1);
    if segLen < 1e-6
        collision = checkPointCollisionLocal(p1, obstacles, safetyDist);
        return;
    end

    n = max(2, ceil(segLen / max(30, safetyDist / 3)));
    for k = 0:n
        p = p1 + (k / n) * (p2 - p1);
        if checkPointCollisionLocal(p, obstacles, safetyDist)
            collision = true;
            return;
        end
    end
end

function collision = checkPathCollisionLocal(path, obstacles, safetyDist)
%CHECKPATHCOLLISIONLOCAL 检查整条 8方向执行路径是否与障碍距离过近。
    collision = false;
    if isempty(path) || isempty(obstacles)
        return;
    end
    if size(path, 1) == 1
        collision = checkPointCollisionLocal(path(1,:), obstacles, safetyDist);
        return;
    end
    for i = 1:size(path, 1)-1
        if checkSegmentCollisionLocal(path(i,:), path(i+1,:), obstacles, safetyDist)
            collision = true;
            return;
        end
    end
end

function collision = checkPointCollisionLocal(point, obstacles, safetyDist)
    collision = any(sum((obstacles - point).^2, 2) < safetyDist^2);
end

function qExec = buildMecanumExecutablePath(qRaw, stepLen)
%BUILDMECANUMEXECUTABLEPATH 将任意路径转换为 8方向麦轮可执行折线。
    if isempty(qRaw) || size(qRaw, 1) < 2
        qExec = qRaw;
        return;
    end

    qExec = qRaw(1,:);
    for i = 1:size(qRaw, 1)-1
        segmentExec = decomposeSegmentToMecanumDirs(qExec(end,:), qRaw(i+1,:), stepLen);
        if ~isempty(segmentExec)
            qExec = [qExec; segmentExec]; %#ok<AGROW>
        end
    end
    qExec = removeNearDuplicatePoints(qExec, 1e-6);
end

function pts = decomposeSegmentToMecanumDirs(p0, p1, stepLen)
    dx = p1(1) - p0(1);
    dy = p1(2) - p0(2);
    sx = sign(dx);
    sy = sign(dy);
    ax = abs(dx);
    ay = abs(dy);
    pts = [];
    p = p0;

    diagLen = min(ax, ay);
    while diagLen > 1e-6
        d = min(stepLen, diagLen);
        p = p + [sx*d, sy*d];
        pts = [pts; p]; %#ok<AGROW>
        diagLen = diagLen - d;
    end

    remainX = ax - min(ax, ay);
    while remainX > 1e-6
        d = min(stepLen, remainX);
        p = p + [sx*d, 0];
        pts = [pts; p]; %#ok<AGROW>
        remainX = remainX - d;
    end

    remainY = ay - min(ax, ay);
    while remainY > 1e-6
        d = min(stepLen, remainY);
        p = p + [0, sy*d];
        pts = [pts; p]; %#ok<AGROW>
        remainY = remainY - d;
    end
end

function q2 = removeNearDuplicatePoints(q, threshold)
    if isempty(q)
        q2 = q;
        return;
    end
    q2 = q(1,:);
    for i = 2:size(q, 1)
        if norm(q(i,:) - q2(end,:)) > threshold
            q2 = [q2; q(i,:)]; %#ok<AGROW>
        end
    end
end

function localPath = pathToLocal(path, currentPos, headingRad)
    localPath = worldToLocalPoints(path, currentPos, headingRad);
end

function [newPos, newHeading] = estimatePoseFromState(currentPos, headingRad, state, speedCmd, dt, speedScale, turnScale, S)
%ESTIMATEPOSEFROMSTATE Open-loop pose estimate from the previous command.
    newPos = currentPos;
    newHeading = headingRad;
    if isempty(state) || state == S.stop || speedCmd <= 0
        return;
    end

    if state == S.rotLeft
        newHeading = wrapAnglePi(headingRad + deg2rad(double(speedCmd) * turnScale * dt));
        return;
    elseif state == S.rotRight
        newHeading = wrapAnglePi(headingRad - deg2rad(double(speedCmd) * turnScale * dt));
        return;
    end

    localDir = driveStateToLocalUnitVector(state, S);
    if norm(localDir) < 1e-6
        return;
    end
    newPos = currentPos + localToWorldVector(localDir, headingRad) * double(speedCmd) * speedScale * dt;
end

function u = driveStateToLocalUnitVector(state, S)
    if state == S.fwd
        u = [0, 1];
    elseif state == S.back
        u = [0, -1];
    else
        u = [0, 0];
    end
end

function ptsWorld = localToWorldPoints(ptsLocal, currentPos, headingRad)
    if isempty(ptsLocal)
        ptsWorld = zeros(0, 2);
        return;
    end
    R = headingRotation(headingRad);
    ptsWorld = ptsLocal * R.' + currentPos;
end

function ptsLocal = worldToLocalPoints(ptsWorld, currentPos, headingRad)
    if isempty(ptsWorld)
        ptsLocal = zeros(0, 2);
        return;
    end
    R = headingRotation(headingRad);
    ptsLocal = (ptsWorld - currentPos) * R;
end

function vWorld = localToWorldVector(vLocal, headingRad)
    R = headingRotation(headingRad);
    vWorld = vLocal * R.';
end

function R = headingRotation(headingRad)
    c = cos(headingRad);
    s = sin(headingRad);
    R = [c, -s; s, c];
end

function a = wrapAnglePi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function target = choosePathTarget(execPath, currentPos, headingRad, lookaheadDist)
    target = [];
    if isempty(execPath) || size(execPath, 1) < 2
        return;
    end

    localPath = worldToLocalPoints(execPath, currentPos, headingRad);
    d = sqrt(sum(localPath.^2, 2));
    candidates = find(d >= lookaheadDist & localPath(:,2) > -50, 1, 'first');
    if isempty(candidates)
        candidates = size(localPath, 1);
    end
    target = localPath(candidates, :);
    if norm(target) < 1e-6
        target = [];
    end
end

function state = vectorToTurnDriveState(v, S, turnDeadbandDeg, reverseHeadingDeg)
%VECTORTOTURNDRIVESTATE Convert a local target vector to forward/back/turn command.
    state = [];
    if isempty(v) || norm(v) < 1e-6
        return;
    end

    dx = v(1);
    dy = v(2);
    targetAngleDeg = rad2deg(atan2(dx, dy));
    if abs(targetAngleDeg) <= turnDeadbandDeg
        state = S.fwd;
    elseif abs(targetAngleDeg) >= reverseHeadingDeg
        state = S.back;
    elseif targetAngleDeg < 0
        state = S.rotLeft;
    else
        state = S.rotRight;
    end
end

function txt = stateToText(state, S)
    if state == S.fwd
        txt = 'forward';
    elseif state == S.back
        txt = 'back';
    elseif state == S.rotLeft
        txt = 'turn_left';
    elseif state == S.rotRight
        txt = 'turn_right';
    else
        txt = 'stop';
    end
end

function obs = safeObstacles(obs)
    if isempty(obs)
        obs = zeros(0, 2);
        return;
    end
    obs = double(obs);
    if size(obs, 2) ~= 2
        obs = reshape(obs, [], 2);
    end
    obs = obs(all(isfinite(obs), 2), :);
end

function goal = chooseRollingWorldGoal(currentPos, globalGoal, localGoalDist)
    toGoal = globalGoal - currentPos;
    d = norm(toGoal);
    if d <= localGoalDist || d < 1e-6
        goal = globalGoal;
    else
        goal = currentPos + toGoal / d * localGoalDist;
    end
end

function setHeadingArrow(h, origin, headingRad, len)
    if ~isgraphics(h), return; end
    tip = localToWorldVector([0, len], headingRad);
    set(h, 'XData', origin(1), 'YData', origin(2), 'UData', tip(1), 'VData', tip(2));
end

function setPlanWorldView(ax, hInfo, currentPos, globalGoal, pp)
    if ~isgraphics(ax), return; end
    xMin = min(currentPos(1) + pp.xLimit(1), globalGoal(1) - 120);
    xMax = max(currentPos(1) + pp.xLimit(2), globalGoal(1) + 120);
    yMin = min(currentPos(2) + pp.yLimit(1), globalGoal(2) - 120);
    yMax = max(currentPos(2) + pp.yLimit(2), globalGoal(2) + 120);
    xlim(ax, [xMin, xMax]);
    ylim(ax, [yMin, yMax]);
    if isgraphics(hInfo)
        set(hInfo, 'Position', [xMin + 20, yMax - 35, 0]);
    end
end

function setPoseInfoText(h, currentPos, headingRad, distToGoal, goalReached, reachDist)
    if ~isgraphics(h), return; end
    statusText = 'NO';
    if goalReached
        statusText = 'YES';
    end
    info = sprintf(['pos = (%.0f, %.0f) mm\n' ...
        'heading = %.1f deg\n' ...
        'distGoal = %.0f mm\n' ...
        'reach <= %.0f mm : %s'], ...
        currentPos(1), currentPos(2), rad2deg(headingRad), ...
        distToGoal, reachDist, statusText);
    set(h, 'String', info);
end

function setScatterPoints(h, pts)
    if ~isgraphics(h), return; end
    if isempty(pts)
        set(h, 'XData', nan, 'YData', nan);
    else
        set(h, 'XData', pts(:,1), 'YData', pts(:,2));
    end
end

function setPathLine(h, path)
    if ~isgraphics(h), return; end
    if isempty(path)
        set(h, 'XData', nan, 'YData', nan);
    else
        set(h, 'XData', path(:,1), 'YData', path(:,2));
    end
end

function setPointMarker(h, point)
    if ~isgraphics(h), return; end
    if isempty(point)
        set(h, 'XData', nan, 'YData', nan);
    else
        set(h, 'XData', point(1), 'YData', point(2));
    end
end

function [action, state, targetSpeed, dbg, st] = decidePlannedFuzzyOmniAvoidance(mx, my, pathTarget, p, fz, S, flip, st, dt)
%DECIDEPLANNEDFUZZYOMNIAVOIDANCE Follow planned path using only forward/back/turn states.
    [action, state, targetSpeed, dbg, st] = decideFuzzyOmniAvoidance(mx, my, p, fz, S, flip, st, dt);

    % 模糊层已决定后退脱困时,优先保证脱困,不让规划路径覆盖
    if state == S.back && st.backupTimer > 0
        return;
    end

    if isempty(pathTarget) || dbg.panic > 0.35 || state == S.stop
        return;
    end

    plannedState = vectorToTurnDriveState(pathTarget, S, fz.turnDeadbandDeg, fz.reverseHeadingDeg);
    if ~isempty(plannedState)
        state = plannedState;
        action = ['path_' stateToText(state, S)];
        if state == S.rotLeft || state == S.rotRight
            targetSpeed = min(targetSpeed, fz.turnSpeed);
        elseif state == S.back
            targetSpeed = min(targetSpeed, fz.backupSpeed);
        end
    end
end

function [action, state, targetSpeed, dbg, st] = decideFuzzyOmniAvoidance(mx, my, p, fz, S, flip, st, dt)
%DECIDEFUZZYOMNIAVOIDANCE 前进/后退/转向避障,带"卡死->后退脱困"恢复。
    action = 'forward';
    state = S.fwd;
    targetSpeed = st.cruiseSpeed;
    dbg = struct('nearest', inf, 'leftClear', inf, 'rightClear', inf, ...
        'hazard', 0, 'panic', 0, 'rateRisk', 0, 'sideBias', 0);

    ahead = my > 0 & my < p.senseRange;
    mx = mx(ahead);
    my = my(ahead);
    st.stateAge    = st.stateAge + dt;
    st.avoidAge    = st.avoidAge + dt;
    st.backupTimer = max(st.backupTimer - dt, 0);

    % 前方无障碍:清空脱困/后退状态,正常前进
    if isempty(my)
        st.lastNearest = inf;
        st.avoidSide   = 0;
        st.avoidAge    = inf;
        st.stuckTimer  = 0;
        st.backupTimer = 0;
        [state, st] = holdStateIfNeeded(state, st, fz.stateHoldTime, S);
        return;
    end

    inCorr = abs(mx) < p.corridorHalfWidth;
    if any(inCorr)
        nearest = min(my(inCorr));
    else
        nearest = inf;
    end
    dbg.nearest = nearest;

    leftSel  = mx < 0;
    rightSel = mx > 0;
    if flip
        [leftSel, rightSel] = deal(rightSel, leftSel);
    end
    leftClear  = sideMin(my(leftSel));
    rightClear = sideMin(my(rightSel));
    dbg.leftClear  = leftClear;
    dbg.rightClear = rightClear;

    if isfinite(leftClear) || isfinite(rightClear)
        sideBias = clampValue((leftClear - rightClear) / p.senseRange, -1, 1);
    else
        sideBias = 0;
    end
    dbg.sideBias = sideBias;
    desiredSide = 1;
    if sideBias < 0
        desiredSide = -1;
    end

    if isfinite(nearest)
        hazard = clampValue((p.slowDist - nearest) / max(p.slowDist - p.stopDist, 1), 0, 1);
        panic  = clampValue((p.stopDist - nearest) / max(p.stopDist - fz.emergencyDist, 1), 0, 1);
    else
        hazard = 0;
        panic  = 0;
    end

    if isfinite(nearest) && isfinite(st.lastNearest)
        closingRate = max((st.lastNearest - nearest) / max(dt, 1e-3), 0);
    else
        closingRate = 0;
    end
    rateRisk = clampValue(closingRate / max(fz.rateRiskScale, 1), 0, 1);

    dbg.hazard   = hazard;
    dbg.panic    = panic;
    dbg.rateRisk = rateRisk;

    fuzzyRisk = clampValue(0.65 * hazard + 0.25 * panic + 0.20 * rateRisk, 0, 1);
    targetSpeed = st.cruiseSpeed - fuzzyRisk * (st.cruiseSpeed - fz.minSpeed);

    % ---- 卡死计时:在 stopDist 以内且没有明显远离,就累计"卡住"时间 ----
    if isfinite(nearest) && nearest < p.stopDist
        movingAway = isfinite(st.lastNearest) && (nearest - st.lastNearest) > 3;  % 正在远离
        if movingAway
            st.stuckTimer = 0;
        else
            st.stuckTimer = st.stuckTimer + dt;
        end
    else
        st.stuckTimer = 0;
    end

    % ---- 决策:后退脱困优先 > 转向避让 > 前进 ----
    if st.backupTimer > 0
        % 正在执行已承诺的后退
        action = 'fuzzy_backup_recover';
        state = S.back;
        targetSpeed = min(targetSpeed, fz.backupSpeed);
    elseif (isfinite(nearest) && nearest < fz.emergencyDist) || st.stuckTimer >= fz.stuckTime
        % 极近 或 转向脱不了困 -> 后退让出规划空间(关键修复:打破原地打转死锁)
        action = 'fuzzy_backup_recover';
        state = S.back;
        targetSpeed = min(targetSpeed, fz.backupSpeed);
        st.backupTimer = fz.backupCommitTime;
        st.stuckTimer  = 0;
        st.avoidSide   = 0;
        st.avoidAge    = inf;
    elseif panic > 0.35 || hazard > 0.15
        if st.avoidSide == 0 || st.avoidAge >= fz.avoidCommitTime
            if st.avoidSide ~= desiredSide
                st.avoidAge = 0;
            end
            st.avoidSide = desiredSide;
        end

        if st.avoidAge >= fz.turnBeforeCreepTime && nearest > p.stopDist
            action = 'fuzzy_creep_forward';
            state = S.fwd;
            targetSpeed = min(targetSpeed, fz.minSpeed);
        elseif st.avoidSide >= 0
            action = 'fuzzy_turn_left_locked';
            state = S.rotLeft;
            targetSpeed = min(targetSpeed, fz.turnSpeed);
        else
            action = 'fuzzy_turn_right_locked';
            state = S.rotRight;
            targetSpeed = min(targetSpeed, fz.turnSpeed);
        end
    else
        action = 'forward';
        state = S.fwd;
        targetSpeed = st.cruiseSpeed;
        st.avoidSide = 0;
        st.avoidAge  = inf;
    end

    targetSpeed = clampValue(targetSpeed, 0, st.cruiseSpeed);
    [state, st] = holdStateIfNeeded(state, st, fz.stateHoldTime, S);
    st.lastNearest = nearest;
end

function [state, st] = holdStateIfNeeded(candidateState, st, holdTime, S)
%HOLDSTATEIFNEEDED Reduce left/right turn chatter between adjacent frames.
    turnStates = [S.rotLeft S.rotRight];
    if st.stateAge < holdTime && ismember(st.lastState, turnStates) && ...
            ismember(candidateState, turnStates) && candidateState ~= st.lastState
        state = st.lastState;
        return;
    end
    state = candidateState;
    if state ~= st.lastState
        st.lastState = state;
        st.stateAge = 0;
    end
end

function y = rampToward(x, target, maxStep)
%RAMPTOWARD 按最大步长逼近目标速度,避免 C 指令突变。
    delta = clampValue(target - x, -maxStep, maxStep);
    y = x + delta;
end

function y = clampValue(x, lo, hi)
    y = max(lo, min(hi, x));
end

function d = sideMin(v)
    if isempty(v), d = inf; else, d = min(v); end
end

% ---- 运动层:把指令字符串写入小车 BLE 特征 ----
function sendCmd(txChar, str)
%SENDCMD 向小车发送一条协议指令字符串,如 'A|2|$' 或 'C|30|$'。
    write(txChar, uint8(char(str)), "WithoutResponse");
    % 注:若你的特征不支持 write-without-response,改为 write(txChar, uint8(char(str)));
end

function txChar = getTxCharacteristic(car)
%GETTXCHARACTERISTIC 唯一适配点:返回一个可 write 的 FFE1 特征句柄。
%  默认从控制类的 ble 对象 car.Device 上获取 FFE0/FFE1 特征。
%  若你的控制类本身有发原始指令的方法,可改用它(并相应改 sendCmd)。
    txChar = characteristic(car.Device, "FFE0", "FFE1");
end

function safeShutdown(car)
    try, car.stop();  catch, end
    try, delete(car); catch, end
end





