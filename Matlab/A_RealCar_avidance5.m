%% 激光建图 + HC-08 四方向小车联合避障测试(A5:严格前后左右运动)
%  Real fisheye laser mapping + HC-08 Mecanum car -> omnidirectional avoidance.
%
%  工作流程 / Pipeline:
%    snapshot -> extractRealRedLaser -> mapping -> 障碍判断 -> 发送 A|n|$ 指令
%
%  运动模型 / Motion model (来自 app_control.ino):
%    指令 "设定即保持":每帧发一条方向指令,小车保持该方向到下一帧。
%    A|<state>|$  运动;  C|<speed>|$  调速;  A|8|$  停止。
%    A5 state白名单: 2=前 6=后 0=左移 4=右移 8=停
%    全程不自转(car head 朝向不变),激光/摄像头始终朝前。
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

camX = 2.96;   camY = 2.35;   camZ = 1.06;
lasX = 0;  lasY = 2;  las_dist = 205;
CVsyst_rot = 0;  CVsyst_x = 0;  CVsyst_y = 0;

MIN_RED_EXCESS             = 18;
MIN_RED_VALUE              = 90;
MIN_HSV_SATURATION         = 0.06;
MIN_HSV_VALUE              = 0.80;
MIN_COMPONENT_AREA         = 45;
MAX_COMPONENT_AREA         = 2000;
MIN_COMPONENT_ECCENTRICITY = 0.20;
MIN_COMPONENT_MAJOR_AXIS   = 25;

%% ===== 避障决策参数 =====
%  激光地图坐标系: map_y = 前方距离(mm), map_x = 横向偏移(mm,负=左 正=右)
%  以下所有距离/宽度阈值单位均为 mm。
av.senseRange        = 700;   % 只关注前方 0~600mm 的障碍点
av.corridorHalfWidth = 150;   % 正前方“走廊”半宽(mm),左右各 150mm,总宽 300mm
av.slowDist          = 450;   % 进入此距离:斜向前进避让(保持前进)
av.stopDist          = 250;   % 进入此距离:纯侧移让开走廊(必要时后退)

DRIVE_SPEED          = 45;    % 巡航速度(0~100);低于电机可靠起步区会降低轨迹准确性

%% ===== 障碍风险与速度参数(不改小车固件) =====
% 障碍点只影响停车与速度，运动方向严格来自四方向路径。
% 输出仍使用 C|speed|$ 与 A|state|$ 协议。
fz.minSpeed          = 25;    % TT 电机可靠起步附近的最低速度
fz.avoidSpeed        = 26;    % 路径附近有障碍时的降速值
fz.emergencyDist     = 50;   % 极近距离且两侧都堵时停车
fz.rateRiskScale     = 320;   % 障碍快速接近时的风险归一化尺度(mm/s)
fz.speedRateLimit    = 15;    % 降低速度指令变化率,优先保证动作稳定
fz.speedCmdDeadband  = 3;     % 减少 BLE 速度指令密度
fz.motorStartSpeed   = 25;    % 实车可靠起步的最小速度命令(不是0~255硬件PWM)
fz.reverseBrakeTime  = 0.30;  % 前后或左右反向前先停车，避免车轮立即反转

%% ===== 局部路径规划参数(RRT* -> 四方向可执行路径) =====
% 坐标系: x<0 左, x>0 右, y>0 前。每帧以小车当前位置 [0,0] 规划到前方局部目标。
pp.enabled           = true;
pp.goal              = [-10, 650];   % local goal relative to current position, mm
pp.replanInterval    = 1.20;       % 重规划最小间隔,避免连续随机改路
pp.safetyDistList    = [95 85 75];  % 车体外接半径约120mm,另留25~50mm余量
pp.execSafetyExtra   = 10;             % 离散折线路径的附加余量
pp.replanSafetyDist  = 80;            % 剩余路径低于此净空时重规划
pp.pathBlockHysteresis = 20;            % 防止25mm栅格抖动立即否决刚生成的路径
pp.minPlanningClearance = 55;         % 小于此值时已接近车体外接半径
pp.minRrtSafetyDist  = 80;            % 不允许小于车体外接半径
pp.execStepLen       = 60;         % 较短执行段,减小开环过冲
pp.maxObstaclePoints = 260;        % RRT 使用的障碍点上限,保证实时性
pp.waypointReachDist = 30;         % 当前节点到达阈值,单位 mm
pp.cornerApproachDist = 90;        % 接近折点时降速,减小开环过冲
pp.cardinalOnly       = true;     % A5固定为4方向，不提供8方向开关
assert(pp.cardinalOnly, 'A5必须保持strict-4-dir模式。');
pp.xLimit            = [-450 450];
pp.yLimit            = [-80 850];
pp.rrtMaxIter        = 700;
pp.rrtAttempts       = 2;          % 每个安全距离生成多条候选后择优
pp.rrtStep           = 100;
pp.rrtGoalTolerance  = 90;
pp.rrtNeighborRadius = 300;
pp.turnPenalty       = 45;         % 抑制不必要折点,提高实车可执行性
lastRawPath          = zeros(0, 2);
lastExecPath         = zeros(0, 2);
plannerStatus        = "init";
plannerSafetyDist    = NaN;

%% ===== 开环当前位置估计(用于让重规划从当前位置开始) =====
% 与 app_control2 的轮速归一化及 map(command,0,100,50,255) 保持一致。
% scale = 实测距离/程序显示距离；完成定距测试后分别修改以下系数。
pose.wheelDiameterMm = 60;
pose.loadedWheelRpm = 64;
pose.maxCommandSpeed = 100;
pose.maxMotorPwm = 255;
pose.firmwareMinPwm = 50;
pose.maxWheelSpeedMmps = ...
    pose.loadedWheelRpm * pi * pose.wheelDiameterMm / 60;
pose.forwardScale = 1.00;
pose.backwardScale = 1.00;
pose.lateralScale = 1.00;
currentPos = [0, 0];


% Temporal obstacle memory. One hit per grid cell per frame prevents a thick
% laser line from being mistaken for repeated observations.
mem.gridResolution = 25;
mem.keepFrames = 10;
mem.confirmHits = 2;
obstacleHistory = zeros(0, 3);  % [gridX, gridY, frame]
confirmedWorldObstacles = zeros(0, 2);

% Laser reflections inside this rectangle belong to the 164 x 176 mm car.
selfMask.halfWidth = 105;
selfMask.rearY = -110;
selfMask.frontY = 105;

ui.displayEveryN = 2;
ui.planEveryN = 2;

%  小车运动 state 编码(对应固件 Rockerandgravity_Task)
S.fwd   = 2;  S.back  = 6;
S.left  = 0;  S.right = 4;   % 纯侧移(平移,不自转)
S.stop  = 8;
lastMotionState = S.stop;
lastMotionSpeed = 0;
motionGuard = initCardinalMotionGuard(S, fz.reverseBrakeTime);
assertCardinalPlannerSelfTest(pp, S);

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

riskState = initObstacleRiskState(DRIVE_SPEED, fz);
currentSpeed = DRIVE_SPEED;
lastSentSpeed = round(currentSpeed);

sendCmd(txChar, sprintf('C|%d|$', lastSentSpeed));       % 设速度
sendCmd(txChar, sprintf('A|%d|$', S.stop));              % 先停住
lastSentState = S.stop;
stateSendClock = tic;
motionClock = tic;

fprintf('[INIT] 相机 webcam(%d)=%s  分辨率=%s\n', ...
    CAMERA_INDEX, string(available_cameras{CAMERA_INDEX}), camera_device.Resolution);
fprintf('[INIT] 标定=%s  激光 las_dist=%.1f mm\n', CALIBRATION_FILE, las_dist);
fprintf('[INIT] 避障: stop<%.0f  slow<%.0f  走廊半宽=%.0f  速度=%d  全向无自转\n', ...
    av.stopDist, av.slowDist, av.corridorHalfWidth, DRIVE_SPEED);
fprintf('[INIT] 模糊避障: minSpeed=%d avoidSpeed=%d emergency<%.0f speedRate=%.0f/s\n', ...
    fz.minSpeed, fz.avoidSpeed, fz.emergencyDist, fz.speedRateLimit);
fprintf('[INIT] A5路径执行: strict-4-dir states=[0 2 4 6], 激光不参与定位\n');
predictedForwardSpeed = norm(estimateStateDisplacement(S.fwd, DRIVE_SPEED, 1, pose, S));
predictedLateralSpeed = norm(estimateStateDisplacement(S.left, DRIVE_SPEED, 1, pose, S));
fprintf('[INIT] 开环速度估计: forward=%.1f lateral=%.1f mm/s  scales=[%.2f %.2f %.2f]\n', ...
    predictedForwardSpeed, predictedLateralSpeed, ...
    pose.forwardScale, pose.backwardScale, pose.lateralScale);

input('小车已连接。建议先架空车轮。按 Enter 开始避障测试,或 Ctrl+C 取消。', 's');

%% ===== 首帧与界面 =====
first_image = snapshot(camera_device);
first_mask  = false(size(first_image, 1), size(first_image, 2));

fig = figure('Name', 'Laser Mapping + Strict 4-Direction Avoidance', 'NumberTitle', 'off');

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
h_raw_path = plot(ax_map, nan, nan, 'c--o', 'LineWidth', 1.6, ...
    'MarkerSize', 3);
h_exec_path = plot(ax_map, nan, nan, 'm-', 'LineWidth', 2.0);
h_path_target = plot(ax_map, nan, nan, 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);
uistack(h_raw_path, 'top');
uistack(h_path_target, 'top');
hold(ax_map, 'off');
axis(ax_map, 'equal'); grid(ax_map, 'on');
xlim(ax_map, [-450, 450]); ylim(ax_map, [-100, 900]);
xlabel(ax_map, 'X / mm'); ylabel(ax_map, 'Y / mm');
title(ax_map, 'Laser Mapping');

fig_plan = figure('Name', 'Local Cardinal + RRT* Path Planning', 'NumberTitle', 'off');
ax_plan = axes(fig_plan);
h_plan_obs = scatter(ax_plan, nan, nan, 12, 'red', 'filled'); hold(ax_plan, 'on');
h_plan_raw = plot(ax_plan, nan, nan, 'c--o', 'LineWidth', 1.8, ...
    'MarkerSize', 4);
h_plan_exec = plot(ax_plan, nan, nan, 'm-', 'LineWidth', 2.0);
h_plan_target = plot(ax_plan, nan, nan, 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
uistack(h_plan_raw, 'top');
uistack(h_plan_target, 'top');
h_plan_car = plot(ax_plan, 0, 0, 'g+', 'MarkerSize', 10, 'LineWidth', 1.5);
h_plan_goal = plot(ax_plan, pp.goal(1), pp.goal(2), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
axis(ax_plan, 'equal'); grid(ax_plan, 'on');
xlim(ax_plan, pp.xLimit); ylim(ax_plan, pp.yLimit);
xlabel(ax_plan, 'X / mm'); ylabel(ax_plan, 'Y / mm');
title(ax_plan, 'Local path planning');
legend(ax_plan, [h_plan_obs h_plan_raw h_plan_exec h_plan_target ...
    h_plan_car h_plan_goal], ...
    {'Obstacles','Raw RRT* reference','4-dir command path','Next target','Car','Goal'}, ...
    'Location', 'best');
hold(ax_plan, 'off');

frame_count = 0;
valid_frame_count = 0;
loop_clock = tic;
frame_clock = tic;
plan_clock = tic;
lastPlanAttemptFrame = 0;

%% ===== 主循环 =====
while isvalid(fig)
    try
        frame_count = frame_count + 1;
        dt = max(toc(frame_clock), 1e-3);
        frame_clock = tic;

        image_rgb = snapshot(camera_device);

        laser_mask = extractRealRedLaser(image_rgb, MIN_RED_EXCESS, ...
            MIN_RED_VALUE, MIN_HSV_SATURATION, MIN_HSV_VALUE, ...
            MIN_COMPONENT_AREA, MAX_COMPONENT_AREA, ...
            MIN_COMPONENT_ECCENTRICITY, MIN_COMPONENT_MAJOR_AXIS);

        [map_x, map_y] = mappingLaserPixelsFast(laser_mask, CVsyst_rot, ...
            CVsyst_y, CVsyst_x, camY, camX, camZ, lasY, lasX, ...
            las_dist, ocam_model);

        % The first output point is the CV origin, matching mapping().
        if numel(map_x) > 1
            map_x = map_x(2:end);
            map_y = map_y(2:end);
        else
            map_x = [];
            map_y = [];
        end
        ok = isfinite(map_x) & isfinite(map_y);
        map_x = map_x(ok);
        map_y = map_y(ok);

        % Remove laser returns from the chassis and wheels. Points in this
        % footprint cannot be external obstacles in the car coordinate frame.
        selfReturn = abs(map_x) <= selfMask.halfWidth & ...
            map_y >= selfMask.rearY & map_y <= selfMask.frontY;
        selfRemovedCount = nnz(selfReturn);
        map_x = map_x(~selfReturn);
        map_y = map_y(~selfReturn);

        % A5:激光点只用于障碍建图，不参与位置或旋转误差估计。
        localObstacles = [map_x(:), map_y(:)];
        worldObstacles = localObstacles + currentPos;
        [obstacleHistory, confirmedWorldObstacles, currentWorldCells] = ...
            updateTemporalObstacles(obstacleHistory, worldObstacles, ...
            currentPos, frame_count, mem, pp);
        plannerWorldObstacles = mergeObstacleSets( ...
            confirmedWorldObstacles, currentWorldCells);
        plannerLocalObstacles = plannerWorldObstacles - currentPos;

        % Use recent confirmed obstacles for emergency control when the laser
        % mask briefly becomes empty. A genuinely clear scene remains clear
        % after the short memory expires.
        controlLocalObstacles = localObstacles;
        laserDropoutHold = false;
        if isempty(controlLocalObstacles) && ~isempty(plannerLocalObstacles)
            visible = plannerLocalObstacles(:,2) > 0 & ...
                plannerLocalObstacles(:,2) < av.senseRange;
            controlLocalObstacles = plannerLocalObstacles(visible, :);
            laserDropoutHold = ~isempty(controlLocalObstacles);
        end

        % Replan only when the remaining path is blocked or exhausted.
        rawPath = lastRawPath;
        execPath = lastExecPath;
        remainingPath = remainingPathFromCurrent(execPath, currentPos, pp.waypointReachDist);
        pathCollisionDist = activePathCollisionDistance(pp, plannerSafetyDist);
        pathBlocked = ~isempty(remainingPath) && ...
            checkPathCollisionLocal(remainingPath, plannerWorldObstacles, ...
            pathCollisionDist);
        pathExhausted = size(remainingPath, 1) < 2;
        startClearance = nearestObstacleDistance(currentPos, plannerWorldObstacles);
        plannerCanStart = startClearance >= pp.minPlanningClearance;
        replanDue = pathExhausted || pathBlocked;
        planChanged = false;

        if pp.enabled && replanDue && ...
                (lastPlanAttemptFrame == 0 || toc(plan_clock) >= pp.replanInterval)
            % Stop before a potentially expensive plan and close odometry now.
            [currentPos, motionClock] = updateOpenLoopPose( ...
                currentPos, motionClock, lastMotionState, lastMotionSpeed, pose, S);
            if lastSentState ~= S.stop
                sendCmd(txChar, sprintf('A|%d|$', S.stop));
                lastSentState = S.stop;
                stateSendClock = tic;
            end
            lastMotionState = S.stop;
            lastMotionSpeed = 0;
            motionClock = tic;

            if plannerCanStart
                worldGoal = currentPos + pp.goal;
                planParams = adaptPlanningClearance(pp, startClearance);
                [newRawPath, newExecPath, newStatus, newSafetyDist] = ...
                    planLocalCardinalPath(plannerWorldObstacles, currentPos, ...
                    worldGoal, planParams);
                if ~isempty(newExecPath)
                    lastRawPath = newRawPath;
                    lastExecPath = newExecPath;
                    plannerStatus = newStatus;
                    plannerSafetyDist = newSafetyDist;
                    planChanged = true;
                elseif isempty(lastExecPath)
                    plannerStatus = "failed_no_path";
                    plannerSafetyDist = newSafetyDist;
                elseif pathBlocked

                    plannerStatus = "failed_blocked";
                else
                    plannerStatus = "failed_hold";
                end
            else
                plannerStatus = "start_blocked";
            end
            plan_clock = tic;
            lastPlanAttemptFrame = frame_count;
        elseif pathBlocked
            plannerStatus = "blocked_wait";
        elseif laserDropoutHold
            plannerStatus = "dropout_hold";
        end

        rawPath = lastRawPath;
        execPath = lastExecPath;
        remainingPath = remainingPathFromCurrent(execPath, currentPos, pp.waypointReachDist);
        pathCollisionDist = activePathCollisionDistance(pp, plannerSafetyDist);
        pathBlocked = ~isempty(remainingPath) && ...
            checkPathCollisionLocal(remainingPath, plannerWorldObstacles, ...
            pathCollisionDist);
        rawDisplayPath = remainingPathFromCurrent(rawPath, currentPos, pp.waypointReachDist);
        rawPathLocal = rawDisplayPath - currentPos;
        execPathLocal = remainingPath - currentPos;
        pathTarget = choosePathTarget(remainingPath, currentPos);
        if pathBlocked || ~plannerCanStart
            pathTarget = [];
        end

        % 路径优先控制:无有效路径时停车等待重规划,不再由模糊方向长期摸索。
        if isempty(controlLocalObstacles)
            controlX = [];
            controlY = [];
        else
            controlX = controlLocalObstacles(:,1);
            controlY = controlLocalObstacles(:,2);
        end
        [action, state, targetSpeed, dbg, riskState] = ...
            decidePathPriorityCardinalControl(controlX, controlY, ...
            pathTarget, av, fz, pp, S, riskState, dt);
        dbg.planStatus = plannerStatus;
        dbg.planSafetyDist = plannerSafetyDist;

        if ~ismember(state, [S.left S.fwd S.right S.back S.stop])
            warning('A5拒绝非四方向状态 A|%d|$。', state);
            action = 'invalid_state_stop';
            state = S.stop;
            targetSpeed = 0;
        end

        [state, targetSpeed, action, motionGuard] = guardCardinalMotion( ...
            state, targetSpeed, action, S, motionGuard);

        [currentPos, motionClock] = updateOpenLoopPose( ...
            currentPos, motionClock, lastMotionState, lastMotionSpeed, pose, S);

        currentSpeed = rampToward(currentSpeed, targetSpeed, fz.speedRateLimit * dt);
        cmdSpeed = round(currentSpeed);
        if state ~= S.stop && cmdSpeed > 0
            cmdSpeed = max(cmdSpeed, fz.motorStartSpeed);
        end
        if abs(cmdSpeed - lastSentSpeed) >= fz.speedCmdDeadband
            sendCmd(txChar, sprintf('C|%d|$', cmdSpeed));
            lastSentSpeed = cmdSpeed;
        end

        % 固件会保持运动状态;仅变化时发送,低频心跳只用于链路保活。
        if state ~= lastSentState || toc(stateSendClock) >= 3.00
            sendCmd(txChar, sprintf('A|%d|$', state));
            lastSentState = state;
            stateSendClock = tic;
        end
        lastMotionState = state;
        lastMotionSpeed = cmdSpeed;
        motionClock = tic;

        % Decimate large image/UI updates; control still runs every frame.
        if mod(frame_count, ui.displayEveryN) == 0
            set(h_image, 'CData', image_rgb);
            set(h_mask,  'CData', laser_mask);
            set(h_map,   'XData', map_x, 'YData', map_y);
            setPathLine(h_raw_path, rawPathLocal);
            setPathLine(h_exec_path, execPathLocal);
            setPointMarker(h_path_target, pathTarget);
        end
        if planChanged || mod(frame_count, ui.planEveryN) == 0
            setPathLine(h_plan_raw, rawPathLocal);
            setPathLine(h_plan_exec, execPathLocal);
            setPointMarker(h_plan_target, pathTarget);
            setScatterPoints(h_plan_obs, plannerLocalObstacles);
        end

        valid_frame_count = valid_frame_count + 1;
        fps = valid_frame_count / max(toc(loop_clock), eps);
        if mod(frame_count, ui.displayEveryN) == 0
            title(ax_map, sprintf('%s | plan=%s sd=%.0f | nearest=%.0fmm | speed=%d | %.1f FPS', ...
                action, char(dbg.planStatus), dbg.planSafetyDist, ...
                dbg.nearest, cmdSpeed, fps), 'Interpreter', 'none');
        end
        if isgraphics(ax_plan) && ...
                (planChanged || mod(frame_count, ui.planEveryN) == 0)
            title(ax_plan, sprintf('4-dir path | %s | pos=(%.0f, %.0f) grid=%d', ...
                char(dbg.planStatus), currentPos(1), currentPos(2), ...
                size(plannerWorldObstacles,1)), ...
                'Interpreter', 'none');
        end

        if mod(frame_count, 10) == 0
            fprintf(['[FRAME %d] action=%s plan=%s sd=%.0f state=%d wheels=%d speed=%d ' ...
                'nearest=%.0f L=%.0f R=%.0f risk=%.2f rate=%.2f pix=%d self=%d grid=%d\n'], ...
                frame_count, action, char(dbg.planStatus), dbg.planSafetyDist, ...
                state, expectedActiveWheelCount(state, S), cmdSpeed, dbg.nearest, dbg.leftClear, dbg.rightClear, ...
                dbg.hazard, dbg.rateRisk, nnz(laser_mask), ...
                selfRemovedCount, size(plannerWorldObstacles,1));
        end

        drawnow limitrate nocallbacks;
    catch ME
        warning('[FRAME %d] %s', frame_count, ME.message);
        try, sendCmd(txChar, sprintf('A|%d|$', S.stop)); catch, end   % 出错立即停车
        pause(0.1);
    end
end

%% ===== 收尾 =====
try, sendCmd(txChar, sprintf('A|%d|$', S.stop)); catch, end
try, delete(car); catch, end
clear camera_device car cleanup;
fprintf('[STOP] 联合避障测试结束并释放 BLE。\n');
%% ================= 局部函数 =================

function [history, confirmed, currentCellsWorld] = updateTemporalObstacles( ...
        history, worldObstacles, currentPos, frameIndex, mem, pp)
%UPDATETEMPORALOBSTACLES Maintain a short-lived, spatially quantized map.
    worldObstacles = safeObstacles(worldObstacles);
    if isempty(worldObstacles)
        currentCells = zeros(0, 2);
    else
        currentCells = unique(round(worldObstacles / mem.gridResolution), ...
            'rows', 'stable');
    end

    if ~isempty(currentCells)
        history = [history; currentCells, ...
            repmat(frameIndex, size(currentCells,1), 1)]; % #ok<AGROW>
    end

    if ~isempty(history)
        recent = history(:,3) >= frameIndex - mem.keepFrames + 1;
        centers = history(:,1:2) * mem.gridResolution;
        rel = centers - currentPos;
        nearby = rel(:,1) >= pp.xLimit(1) - pp.replanSafetyDist & ...
            rel(:,1) <= pp.xLimit(2) + pp.replanSafetyDist & ...
            rel(:,2) >= pp.yLimit(1) - pp.replanSafetyDist & ...
            rel(:,2) <= pp.yLimit(2) + pp.replanSafetyDist;
        history = history(recent & nearby, :);
    end

    currentCellsWorld = currentCells * mem.gridResolution;
    if isempty(history)
        confirmed = zeros(0, 2);
        return;
    end

    [cells, ~, group] = unique(history(:,1:2), 'rows');
    hits = accumarray(group, 1);
    confirmed = cells(hits >= mem.confirmHits, :) * mem.gridResolution;
end

function merged = mergeObstacleSets(a, b)
%MERGEOBSTACLESETS Merge confirmed and current obstacle cells.
    a = safeObstacles(a);
    b = safeObstacles(b);
    if isempty(a) && isempty(b)
        merged = zeros(0, 2);
    else
        merged = unique([a; b], 'rows', 'stable');
    end
end

function remaining = remainingPathFromCurrent(path, currentPos, reachDist)
%REMAININGPATHFROMCURRENT Trim reached or already-passed path segments.
    if isempty(path)
        remaining = zeros(0, 2);
        return;
    end
    if size(path, 1) == 1
        if norm(path(1,:) - currentPos) <= reachDist
            remaining = currentPos;
        else
            remaining = [currentPos; path];
        end
        return;
    end

    bestDistance = inf;
    bestSegment = 1;
    bestProjection = 0;
    for i = 1:size(path, 1) - 1
        segment = path(i+1,:) - path(i,:);
        segmentLength2 = dot(segment, segment);
        if segmentLength2 <= eps
            continue;
        end
        projection = dot(currentPos - path(i,:), segment) / segmentLength2;
        closest = path(i,:) + clampValue(projection, 0, 1) * segment;
        distance = norm(currentPos - closest);
        if distance < bestDistance
            bestDistance = distance;
            bestSegment = i;
            bestProjection = projection;
        end
    end

    nextIndex = bestSegment + 1;
    if bestProjection >= 1 || ...
            norm(path(nextIndex,:) - currentPos) <= reachDist
        nextIndex = nextIndex + 1;
    end
    while nextIndex <= size(path,1) && ...
            norm(path(nextIndex,:) - currentPos) <= reachDist
        nextIndex = nextIndex + 1;
    end

    if nextIndex > size(path,1)
        remaining = currentPos;
    else
        remaining = [currentPos; path(nextIndex:end,:)];
    end
end

function distance = nearestObstacleDistance(point, obstacles)
%NEARESTOBSTACLEDISTANCE Euclidean clearance from a point to obstacle cells.
    if isempty(obstacles)
        distance = inf;
    else
        distance = sqrt(min(sum((obstacles - point).^2, 2)));
    end
end
function st = initObstacleRiskState(cruiseSpeed, ~)
%INITOBSTACLERISKSTATE Store obstacle-risk history used for speed control.
    st.cruiseSpeed = cruiseSpeed;
    st.lastNearest = inf;
end

function guard = initCardinalMotionGuard(S, brakeTime)
%INITCARDINALMOTIONGUARD Require a stop before an opposite command.
    guard.lastMovingState = S.stop;
    guard.pendingState = S.stop;
    guard.pendingClock = tic;
    guard.brakeTime = brakeTime;
end

function safetyDist = activePathCollisionDistance(pp, plannedSafetyDist)
%ACTIVEPATHCOLLISIONDIST Add hysteresis so one grid-cell shift does not stop the car.
    safetyDist = pp.replanSafetyDist;
    if isfinite(plannedSafetyDist)
        safetyDist = min(safetyDist, plannedSafetyDist);
    end
    safetyDist = max(pp.minRrtSafetyDist, ...
        safetyDist - pp.pathBlockHysteresis);
end
function planParams = adaptPlanningClearance(pp, startClearance)
%ADAPTPLANNINGCLEARANCE Permit an escape plan near, but not inside, danger.
    planParams = pp;
    if ~isfinite(startClearance) || startClearance >= ...
            pp.safetyDistList(1) + pp.execSafetyExtra + 10
        return;
    end

    maxRawSafety = floor(startClearance - pp.execSafetyExtra - 10);
    firstSafety = min(pp.safetyDistList(1), maxRawSafety);
    candidates = [firstSafety, firstSafety - 25, firstSafety - 45];
    candidates = max(candidates, pp.minRrtSafetyDist);
    planParams.safetyDistList = unique(candidates, 'stable');
end
function [rawPath, execPath, status, usedSafetyDist] = planLocalCardinalPath(obstacles, start, goal, pp)
%PLANLOCALMECANUMPATH Try deterministic cardinal detours before RRT*.
    status = "failed";
    usedSafetyDist = NaN;
    rawPath = zeros(0, 2);
    execPath = zeros(0, 2);

    start = double(start(:).');
    goal = double(goal(:).');
    obstacles = safeObstacles(obstacles);
    obstacles = filterPlannerObstacles(obstacles, pp, start);

    axisOrders = ["xy", "yx"];  % 两种曼哈顿折线均参与择优

    for safetyDist = pp.safetyDistList
        execSafetyDist = safetyDist + pp.execSafetyExtra;

        % Sparse real laser maps are usually solved faster and more reliably
        % by a deterministic left/right corridor than by random sampling.
        fastCandidates = buildFastCardinalCandidates(start, goal, obstacles, ...
            execSafetyDist, pp);
        [bestRaw, bestExec, ~] = chooseBestExecutableCandidate( ...
            fastCandidates, obstacles, execSafetyDist, pp, axisOrders);
        if ~isempty(bestExec)
            rawPath = bestRaw;
            execPath = bestExec;
            usedSafetyDist = execSafetyDist;
            if size(bestRaw, 1) == 2
                status = "direct_executable";
            else
                status = "cardinal_detour";
            end
            return;
        end

        % Do not spend RRT* iterations on a clearance for which the goal
        % itself is occupied. The next, smaller safety tier may be feasible.
        if checkPointCollisionLocal(goal, obstacles, execSafetyDist)
            continue;
        end

        bestRaw = zeros(0, 2);
        bestExec = zeros(0, 2);
        bestScore = inf;
        for attempt = 1:pp.rrtAttempts
            try
                candidateRaw = rrtStarLocalPath(start, goal, obstacles, safetyDist, pp);
                [candidateBestRaw, candidateBestExec, candidateScore] = ...
                    chooseBestExecutableCandidate({candidateRaw}, obstacles, ...
                    execSafetyDist, pp, axisOrders);
                if candidateScore < bestScore
                    bestScore = candidateScore;
                    bestRaw = candidateBestRaw;
                    bestExec = candidateBestExec;
                end
            catch
                % A failed random attempt is expected; remaining attempts continue.
            end
        end
        if ~isempty(bestExec)
            rawPath = bestRaw;
            execPath = bestExec;
            usedSafetyDist = execSafetyDist;
            status = "rrt_best";
            return;
        end
    end
end

function candidates = buildFastCardinalCandidates(start, goal, obstacles, clearance, pp)
%BUILDFASTCARDINALCANDIDATES Direct and left/right rectangular detours.
    candidates = {};
    direct = [start; goal];
    if ~checkSegmentCollisionLocal(start, goal, obstacles, clearance)
        candidates = {direct};
        return;
    end
    if isempty(obstacles)
        return;
    end

    yLo = min(start(2), goal(2)) - clearance;
    yHi = max(start(2), goal(2)) + clearance;
    relevant = obstacles(:,2) >= yLo & obstacles(:,2) <= yHi;
    obs = obstacles(relevant, :);
    if isempty(obs)
        return;
    end

    sidePadding = clearance + 15;
    xChoices = [min(obs(:,1)) - sidePadding, max(obs(:,1)) + sidePadding];
    xBounds = start(1) + pp.xLimit;
    firstObstacleY = min(obs(:,2));
    stageY = max(start(2), firstObstacleY - clearance - 15);

    for x = xChoices
        if x <= xBounds(1) || x >= xBounds(2)
            continue;
        end
        routes = {
            [start; x start(2); x goal(2); goal], ...
            [start; start(1) stageY; x stageY; x goal(2); goal]};
        for k = 1:numel(routes)
            route = removeNearDuplicatePoints(routes{k}, 1e-6);
            if size(route, 1) >= 2
                candidates{end+1} = route; %#ok<AGROW>
            end
        end
    end
end

function [bestRaw, bestExec, bestScore] = chooseBestExecutableCandidate( ...
        rawCandidates, obstacles, safetyDist, pp, axisOrders)
    bestRaw = zeros(0, 2);
    bestExec = zeros(0, 2);
    bestScore = inf;
    for i = 1:numel(rawCandidates)
        candidateRaw = rawCandidates{i};
        for axisOrder = axisOrders
            candidateExec = buildCardinalExecutablePath(candidateRaw, ...
                pp.execStepLen, axisOrder);
            if size(candidateExec, 1) < 2 || ...
                    ~isCardinalPath(candidateExec) || ...
                    checkPathCollisionLocal(candidateExec, obstacles, safetyDist)
                continue;
            end
            score = executablePathScore(candidateExec, pp.turnPenalty);
            if score < bestScore
                bestScore = score;
                bestRaw = candidateRaw;
                bestExec = candidateExec;
            end
        end
    end
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
        % Sort before decimation so retained points cover the planning area.
        obstacles = sortrows(obstacles, [2 1]);
        idx = unique(round(linspace(1, size(obstacles,1), ...
            pp.maxObstaclePoints)));
        obstacles = obstacles(idx, :);
    end
end

function path = rrtStarLocalPath(start, goal, obstacles, safetyDist, pp)
%RRTSTARLOCALPATH Continue optimization after the first solution is found.
    obstacles = safeObstacles(obstacles);
    xLimit = start(1) + pp.xLimit;
    yLimit = start(2) + pp.yLimit;

    if ~checkSegmentCollisionLocal(start, goal, obstacles, safetyDist)
        path = [start; goal];
        return;
    end

    tree = start;
    parent = -1;
    cost = 0;
    goalCandidates = zeros(0, 1);

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
            if n == 1 || isAncestorLocal(n, newIdx, parent)
                continue;
            end
            newCost = cost(newIdx) + norm(tree(n,:) - newNode);
            if newCost < cost(n) && ...
                    ~checkSegmentCollisionLocal(newNode, tree(n,:), obstacles, safetyDist)
                parent(n) = newIdx;
                cost = refreshSubtreeCosts(tree, parent, cost, n);
            end
        end
        if norm(newNode - goal) <= pp.rrtGoalTolerance && ...
                ~checkSegmentCollisionLocal(newNode, goal, obstacles, safetyDist)
            goalCandidates(end+1,1) = newIdx; %#ok<AGROW>
        end
    end

    if isempty(goalCandidates)
        error('local RRT path not found');
    end

    goalCost = cost(goalCandidates) + ...
        sqrt(sum((tree(goalCandidates,:) - goal).^2, 2));
    [~, bestGoal] = min(goalCost);
    path = backtrackLocalPath(tree, parent, goalCandidates(bestGoal));
    path = [path; goal];
    path = pruneLocalPath(path, obstacles, safetyDist);
end

function path = backtrackLocalPath(tree, parent, idx)
    path = [];
    while idx > 0
        path = [tree(idx,:); path]; %#ok<AGROW>
        idx = parent(idx);
    end
end
function tf = isAncestorLocal(candidate, node, parent)
    tf = false;
    while node > 0
        if node == candidate
            tf = true;
            return;
        end
        node = parent(node);
    end
end

function cost = refreshSubtreeCosts(tree, parent, cost, root)
    p0 = parent(root);
    cost(root) = cost(p0) + norm(tree(root,:) - tree(p0,:));
    queue = root;
    while ~isempty(queue)
        p = queue(1);
        queue(1) = [];
        children = find(parent == p);
        for child = children(:).'
            cost(child) = cost(p) + norm(tree(child,:) - tree(p,:));
        end
        queue = [queue; children(:)]; %#ok<AGROW>
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

function qExec = buildCardinalExecutablePath(qRaw, stepLen, axisOrder)
%BUILDCARDINALEXECUTABLEPATH Convert every RRT* edge to horizontal/vertical segments.
    if isempty(qRaw) || size(qRaw, 1) < 2
        qExec = qRaw;
        return;
    end

    qExec = qRaw(1,:);
    for i = 1:size(qRaw, 1)-1
        segmentExec = decomposeSegmentToCardinalDirs(qExec(end,:), ...
            qRaw(i+1,:), stepLen, axisOrder);
        if ~isempty(segmentExec)
            qExec = [qExec; segmentExec]; %#ok<AGROW>
        end
    end
    qExec = removeNearDuplicatePoints(qExec, 1e-6);
end

function pts = decomposeSegmentToCardinalDirs(p0, p1, stepLen, axisOrder)
    dx = p1(1) - p0(1);
    dy = p1(2) - p0(2);
    sx = sign(dx);
    sy = sign(dy);
    ax = abs(dx);
    ay = abs(dy);
    pts = [];
    p = p0;

    remainX = ax;
    remainY = ay;
    if axisOrder == "yx"
        axesToRun = [2 1];
    else
        axesToRun = [1 2];
    end
    for axisId = axesToRun
        if axisId == 1
            remaining = remainX;
            unit = [sx, 0];
        else
            remaining = remainY;
            unit = [0, sy];
        end
        while remaining > 1e-6
            d = min(stepLen, remaining);
            p = p + d * unit;
            pts = [pts; p]; %#ok<AGROW>
            remaining = remaining - d;
        end
    end
end

function score = executablePathScore(path, turnPenalty)
    delta = diff(path, 1, 1);
    segmentLength = sqrt(sum(delta.^2, 2));
    direction = sign(delta);
    turns = 0;
    if size(direction, 1) > 1
        turns = nnz(any(direction(2:end,:) ~= direction(1:end-1,:), 2));
    end
    score = sum(segmentLength) + turnPenalty * turns;
end
function valid = isCardinalPath(path)
%ISCARDINALPATH True only when every nonzero segment is horizontal or vertical.
    if size(path,1) < 2
        valid = false;
        return;
    end
    delta = diff(path, 1, 1);
    tolerance = 1e-6;
    valid = all(abs(delta(:,1)) <= tolerance | abs(delta(:,2)) <= tolerance);
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

function [position, clock] = updateOpenLoopPose( ...
        position, clock, state, speed, pose, S)
%UPDATEOPENLOOPPOSE Motor-command dead reckoning only; laser points are not used.
    elapsed = max(toc(clock), 0);
    position = position + estimateStateDisplacement(state, speed, elapsed, pose, S);
    clock = tic;
end
function delta = estimateStateDisplacement(state, speedCmd, dt, pose, S)
%ESTIMATESTATEDISPLACEMENT Match app_control2 normalization and hardware PWM map.
    if isempty(state) || state == S.stop || speedCmd <= 0
        delta = [0, 0];
        return;
    end

    speedCmd = min(max(double(speedCmd), 0), pose.maxCommandSpeed);
    activeMotorCommand = speedCmd / sqrt(2);
    mecanumScale = 1;

    effectivePwm = pose.firmwareMinPwm + ...
        (pose.maxMotorPwm - pose.firmwareMinPwm) * ...
        activeMotorCommand / pose.maxCommandSpeed;
    wheelSpeed = pose.maxWheelSpeedMmps * effectivePwm / pose.maxMotorPwm;
    linearSpeed = wheelSpeed * mecanumScale * motionCalibrationScale(state, pose, S);
    delta = stateToUnitVector(state, S) * linearSpeed * dt;
end
function scale = motionCalibrationScale(state, pose, S)
%MOTIONCALIBRATIONSCALE Direction-specific correction from measured travel.
    if state == S.fwd
        scale = pose.forwardScale;
    elseif state == S.back
        scale = pose.backwardScale;
    elseif ismember(state, [S.left S.right])
        scale = pose.lateralScale;

    else
        scale = 0;
    end
end
function u = stateToUnitVector(state, S)
    if state == S.fwd
        u = [0, 1];
    elseif state == S.back
        u = [0, -1];
    elseif state == S.left
        u = [-1, 0];
    elseif state == S.right
        u = [1, 0];

    else
        u = [0, 0];
    end
end

function target = choosePathTarget(remainingPath, currentPos)
%CHOOSEPATHTARGET Follow the immediate executable segment without cutting corners.
    target = [];
    if isempty(remainingPath) || size(remainingPath,1) < 2
        return;
    end
    target = remainingPath(2,:) - currentPos;
    if norm(target) < 1e-6
        target = [];
    end
end
function state = vectorToCardinalState(v, S)
%VECTORTOCARDINALSTATE Select exactly one of left/forward/right/back.
    state = [];
    if isempty(v) || norm(v) < 1e-6
        return;
    end
    if abs(v(1)) >= abs(v(2))
        if v(1) < 0, state = S.left; else, state = S.right; end
    else
        if v(2) < 0, state = S.back; else, state = S.fwd; end
    end
end
function count = expectedActiveWheelCount(state, S)
    if state == S.stop
        count = 0;
    elseif ismember(state, [S.left S.fwd S.right S.back])
        count = 4;
    else
        count = 0;
    end
end
function txt = stateToText(state, S)
    if state == S.fwd
        txt = 'forward';
    elseif state == S.back
        txt = 'back';
    elseif state == S.left
        txt = 'left';
    elseif state == S.right
        txt = 'right';

    else
        txt = 'stop';
    end
end

function assertCardinalPlannerSelfTest(pp, S)
%ASSERTCARDINALPLANNERSELFTEST Verify path decomposition and command whitelist.
    rawPath = [0, 0; 85, 55; -25, 145];
    for order = ["xy", "yx"]
        executable = buildCardinalExecutablePath(rawPath, pp.execStepLen, order);
        assert(isCardinalPath(executable), ...
            'A5四方向自检失败:可执行路径中仍存在斜线段。');
        assert(norm(executable(end,:) - rawPath(end,:)) < 1e-6, ...
            'A5四方向自检失败:曼哈顿分解改变了路径终点。');
    end
    testVectors = [-1 0; 0 1; 1 0; 0 -1; 2 1; 1 2];
    states = zeros(size(testVectors,1),1);
    for i = 1:size(testVectors,1)
        states(i) = vectorToCardinalState(testVectors(i,:), S);
    end
    assert(all(ismember(states, [S.left S.fwd S.right S.back])), ...
        'A5四方向自检失败:状态映射产生了非四方向指令。');
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

function [action, state, targetSpeed, dbg, st] = decidePathPriorityCardinalControl(mx, my, pathTarget, p, fz, pp, S, st, dt)
%DECIDEPATHPRIORITYCARDINALCONTROL Track a validated four-direction path.
    [dbg, st] = measureObstacleRisk(mx, my, p, fz, st, dt);

    if isempty(pathTarget)
        action = 'wait_for_plan';
        state = S.stop;
        targetSpeed = 0;
        return;
    end

    state = vectorToCardinalState(pathTarget, S);
    if isempty(state)
        action = 'waypoint_stop';
        state = S.stop;
        targetSpeed = 0;
        return;
    end

    if dbg.nearest < fz.emergencyDist
        action = 'emergency_stop';
        state = S.stop;
        targetSpeed = 0;
        return;
    end

    action = ['path_' stateToText(state, S)];
    if dbg.hazard > 0
        minimumMovingSpeed = min(fz.minSpeed, st.cruiseSpeed);
        avoidTarget = clampValue(fz.avoidSpeed, ...
            minimumMovingSpeed, st.cruiseSpeed);
        targetSpeed = st.cruiseSpeed - ...
            dbg.hazard * (st.cruiseSpeed - avoidTarget);
    else
        targetSpeed = st.cruiseSpeed;
    end
    if norm(pathTarget) <= pp.cornerApproachDist
        targetSpeed = min(targetSpeed, fz.minSpeed);
    end
end
function [dbg, st] = measureObstacleRisk(mx, my, p, fz, st, dt)
%MEASUREOBSTACLERISK Compute obstacle metrics without issuing a direction.
    dbg = struct('nearest', inf, 'leftClear', inf, 'rightClear', inf, ...
        'hazard', 0, 'rateRisk', 0);

    ahead = my > 0 & my < p.senseRange;
    mx = mx(ahead);
    my = my(ahead);
    if isempty(my)
        st.lastNearest = inf;
        return;
    end

    inCorr = abs(mx) < p.corridorHalfWidth;
    if any(inCorr)
        nearest = min(my(inCorr));
    else
        nearest = inf;
    end
    dbg.nearest = nearest;

    leftSel = mx < 0;
    rightSel = mx > 0;
    leftClear = sideMin(my(leftSel));
    rightClear = sideMin(my(rightSel));
    dbg.leftClear = leftClear;
    dbg.rightClear = rightClear;

    if isfinite(nearest)
        dbg.hazard = clampValue( ...
            (p.slowDist - nearest) / max(p.slowDist - p.stopDist, 1), 0, 1);
    end
    if isfinite(nearest) && isfinite(st.lastNearest)
        closingRate = max((st.lastNearest - nearest) / max(dt, 1e-3), 0);
    else
        closingRate = 0;
    end
    dbg.rateRisk = clampValue( ...
        closingRate / max(fz.rateRiskScale, 1), 0, 1);
    st.lastNearest = nearest;
end

function [state, targetSpeed, action, guard] = guardCardinalMotion( ...
        requestedState, requestedSpeed, action, S, guard)
%GUARDCARDINALMOTION Insert a stationary interval before direct reversal.
    state = requestedState;
    targetSpeed = requestedSpeed;

    if requestedState == S.stop
        guard.pendingState = S.stop;
        return;
    end
    if guard.lastMovingState == S.stop
        guard.lastMovingState = requestedState;
        return;
    end

    if areOppositeCardinalStates( ...
            requestedState, guard.lastMovingState, S)
        if guard.pendingState ~= requestedState
            guard.pendingState = requestedState;
            guard.pendingClock = tic;
        end
        if toc(guard.pendingClock) < guard.brakeTime
            state = S.stop;
            targetSpeed = 0;
            action = [action '_reverse_brake'];
            return;
        end
    end

    guard.lastMovingState = requestedState;
    guard.pendingState = S.stop;
end

function tf = areOppositeCardinalStates(a, b, S)
%AREOPPOSITECARDINALSTATES True for forward/back or left/right reversal.
    tf = (a == S.fwd && b == S.back) || ...
        (a == S.back && b == S.fwd) || ...
        (a == S.left && b == S.right) || ...
        (a == S.right && b == S.left);
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
%GETTXCHARACTERISTIC Reuse the characteristic opened by the controller.
    if isprop(car, 'TxCharacteristic') && ~isempty(car.TxCharacteristic)
        txChar = car.TxCharacteristic;
    else
        txChar = characteristic(car.Device, "FFE0", "FFE1");
    end
end
function safeShutdown(car)
    try, car.stop();  catch, end
    try, delete(car); catch, end
end
