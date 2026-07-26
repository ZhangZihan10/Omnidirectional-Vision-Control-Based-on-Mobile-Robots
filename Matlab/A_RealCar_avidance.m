%% 激光建图 + HC-08 全向小车 联合避障测试(全向版,零自转)
%  Real fisheye laser mapping + HC-08 Mecanum car -> omnidirectional avoidance.
%
%  工作流程 / Pipeline:
%    snapshot -> extractRealRedLaser -> mapping -> 障碍判断 -> 发送 A|n|$ 指令
%
%  运动模型 / Motion model (来自 app_control.ino):
%    指令 "设定即保持":每帧发一条方向指令,小车保持该方向到下一帧。
%    A|<state>|$  运动;  C|<speed>|$  调速;  A|8|$  停止。
%    state: 2=前 6=后 0=左移 4=右移 1=左前 3=右前 7=左后 5=右后 8=停
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

camX = 2;   camY = 0;   camZ = -2;
lasX = -0.75;  lasY = 0.085;  las_dist = 175;
CVsyst_rot = 0;  CVsyst_x = 0;  CVsyst_y = 0;

MIN_RED_EXCESS             = 18;
MIN_RED_VALUE              = 90;
MIN_HSV_SATURATION         = 0.06;
MIN_HSV_VALUE              = 0.90;
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

DRIVE_SPEED          = 30;    % 行驶速度(0~100,慢一点更安全;斜向时固件自动 /√2)
FLIP_STEERING        = false; % 若小车避让方向反了,改成 true

%  小车运动 state 编码(对应固件 Rockerandgravity_Task)
S.fwd   = 2;  S.back  = 6;
S.left  = 0;  S.right = 4;   % 纯侧移(平移,不自转)
S.fl    = 1;  S.fr    = 3;   % 左前 / 右前 斜向
S.bl    = 7;  S.br    = 5;   % 左后 / 右后 斜向
S.stop  = 8;

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

sendCmd(txChar, sprintf('C|%d|$', round(DRIVE_SPEED)));  % 设速度
sendCmd(txChar, sprintf('A|%d|$', S.stop));              % 先停住

fprintf('[INIT] 相机 webcam(%d)=%s  分辨率=%s\n', ...
    CAMERA_INDEX, string(available_cameras{CAMERA_INDEX}), camera_device.Resolution);
fprintf('[INIT] 标定=%s  激光 las_dist=%.1f mm\n', CALIBRATION_FILE, las_dist);
fprintf('[INIT] 避障: stop<%.0f  slow<%.0f  走廊半宽=%.0f  速度=%d  全向无自转\n', ...
    av.stopDist, av.slowDist, av.corridorHalfWidth, DRIVE_SPEED);

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
hold(ax_map, 'off');
axis(ax_map, 'equal'); grid(ax_map, 'on');
xlim(ax_map, [-300, 300]); ylim(ax_map, [-100, 700]);
xlabel(ax_map, 'X / mm'); ylabel(ax_map, 'Y / mm');
title(ax_map, 'Laser Mapping');

frame_count = 0;
valid_frame_count = 0;
loop_clock = tic;

%% ===== 主循环 =====
while isvalid(fig)
    try
        frame_count = frame_count + 1;

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

        % ---- 决策(全向,零自转)----
        [action, state, dbg] = decideOmniAvoidance(map_x, map_y, av, S, FLIP_STEERING);

        % ---- 执行:发一条方向指令,小车保持到下一帧 ----
        sendCmd(txChar, sprintf('A|%d|$', state));

        % ---- 显示 ----
        set(h_image, 'CData', image_rgb);
        set(h_mask,  'CData', laser_mask);
        set(h_map,   'XData', map_x, 'YData', map_y);

        valid_frame_count = valid_frame_count + 1;
        fps = valid_frame_count / max(toc(loop_clock), eps);
        title(ax_map, sprintf('%s | nearest=%.0fmm | %.1f FPS', ...
            action, dbg.nearest, fps), 'Interpreter', 'none');

        if mod(frame_count, 10) == 0
            fprintf('[FRAME %d] action=%s state=%d nearest=%.0f L=%.0f R=%.0f pix=%d\n', ...
                frame_count, action, state, dbg.nearest, dbg.leftClear, ...
                dbg.rightClear, nnz(laser_mask));
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

function [action, state, dbg] = decideOmniAvoidance(mx, my, p, S, flip)
%DECIDEOMNIAVOIDANCE 全向避障决策(零自转)。
%  坐标(单位 mm): my=前方距离, mx=横向(默认 mx<0 为左, mx>0 为右)。
%  返回: action 文本, state 运动编码, dbg 调试量。
    action = 'forward';  state = S.fwd;
    dbg = struct('nearest', inf, 'leftClear', inf, 'rightClear', inf);

    ahead = my > 0 & my < p.senseRange;
    mx = mx(ahead);  my = my(ahead);
    if isempty(my)
        return;   % 前方无点 -> 视为畅通 -> 直行
    end

    % 正前方走廊内最近障碍距离
    inCorr = abs(mx) < p.corridorHalfWidth;
    if any(inCorr)
        nearest = min(my(inCorr));
    else
        nearest = inf;
    end
    dbg.nearest = nearest;

    % 左右两侧最近障碍距离(距离大 = 更空旷)
    leftSel  = mx < 0;
    rightSel = mx > 0;
    if flip
        [leftSel, rightSel] = deal(rightSel, leftSel);
    end
    leftClear  = sideMin(my(leftSel));
    rightClear = sideMin(my(rightSel));
    dbg.leftClear  = leftClear;
    dbg.rightClear = rightClear;

    leftIsClearer = leftClear >= rightClear;

    if nearest >= p.slowDist
        action = 'forward';  state = S.fwd;            % 畅通:直行
    elseif nearest >= p.stopDist
        % 障碍中距:朝更空一侧斜向前进(保持前进)
        if leftIsClearer
            action = 'veer_front_left';   state = S.fl;
        else
            action = 'veer_front_right';  state = S.fr;
        end
    else
        % 障碍很近:纯侧移让开;两侧都堵则后退
        if max(leftClear, rightClear) < p.stopDist
            action = 'backup';  state = S.back;
        elseif leftIsClearer
            action = 'strafe_left';   state = S.left;
        else
            action = 'strafe_right';  state = S.right;
        end
    end
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