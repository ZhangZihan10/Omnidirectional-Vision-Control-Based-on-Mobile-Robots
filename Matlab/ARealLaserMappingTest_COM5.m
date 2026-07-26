%% 真实环境下激光检测测试 Real fisheye camera laser extraction and mapping test
% Fixed camera test: webcam snapshot -> laser extraction -> mapping.
% Stop the test by closing the figure or pressing Ctrl+C.

clear;
clc;

%cam=webcam(1);
%preview(cam);

%black_image =snapshot(cam);
%图像处理，增强激光线段
% 将图像从RGB转换到HSV
%hsvImage = rgb2hsv(black_image);
%figure;
%imshow(hsvImage);

%% Camera and calibration configuration
CAMERA_INDEX = 2;
CAMERA_RESOLUTION = '1920x1080';
CAMERA_BRIGHTNESS = -30;
CALIBRATION_FILE = 'Omni_Calib_Results_Real2old2.mat';

camX = 2.96;   camY = 2.35;   camZ = 1.06;
lasX = 0;  lasY = 2;  las_dist = 205;
CVsyst_rot = 0;  CVsyst_x = 0;  CVsyst_y = 0;

% Real red-laser extraction parameters. Adjust these first if the laser
% mask contains too much background or misses weak laser pixels.
MIN_RED_EXCESS = 18;
MIN_RED_VALUE = 90;
MIN_HSV_SATURATION = 0.06; 
MIN_HSV_VALUE = 0.80;
MIN_COMPONENT_AREA = 100;
MAX_COMPONENT_AREA = 30000;
MIN_COMPONENT_ECCENTRICITY = 0.20;
MIN_COMPONENT_MAJOR_AXIS = 25;

%% Load real fisheye calibration
calibration = load(CALIBRATION_FILE);
ocam_model = calibration.calib_data.ocam_model;

%% Open real camera
available_cameras = webcamlist;
if CAMERA_INDEX > numel(available_cameras)
    error('webcam(%d) is unavailable. Detected cameras: %s', ...
        CAMERA_INDEX, strjoin(string(available_cameras), ', '));
end

camera_device = webcam(CAMERA_INDEX);

try
    camera_device.Resolution = CAMERA_RESOLUTION;
catch ME
    warning('Unable to set resolution to %s: %s', ...
        CAMERA_RESOLUTION, ME.message);
end

try
    camera_device.Brightness = CAMERA_BRIGHTNESS;
catch ME
    warning('Unable to set camera brightness to %.1f: %s', ...
        CAMERA_BRIGHTNESS, ME.message);
end

% Optional camera tuning. Uncomment these only if the camera supports them.
% camera_device.Exposure = -7;

fprintf('[INIT] Camera: webcam(%d) = %s\n', ...
    CAMERA_INDEX, string(available_cameras{CAMERA_INDEX}));
fprintf('[INIT] Resolution: %s\n', camera_device.Resolution);
fprintf('[INIT] Brightness request: %.1f\n', CAMERA_BRIGHTNESS);
fprintf('[INIT] Calibration: %s\n', CALIBRATION_FILE);
fprintf('[INIT] Camera rotation: camX=%.3f camY=%.3f camZ=%.3f deg\n', ...
    camX, camY, camZ);
fprintf('[INIT] Laser plane: lasX=%.3f lasY=%.3f las_dist=%.3f mm\n', ...
    lasX, lasY, las_dist);

%% First frame and real-time display
first_image = snapshot(camera_device);
first_mask = false(size(first_image, 1), size(first_image, 2));

fig = figure('Name', 'Real Laser Extraction and Mapping - Webcam', ...
    'NumberTitle', 'off');

ax_image = subplot(1,3,1);
h_image = imshow(first_image, 'Parent', ax_image);
title(ax_image, 'Real Fisheye Image');

ax_mask = subplot(1,3,2);
h_mask = imshow(first_mask, 'Parent', ax_mask);
title(ax_mask, 'Extracted Laser');

ax_map = subplot(1,3,3);
h_map = scatter(ax_map, nan, nan, 8, 'filled');
hold(ax_map, 'on');
plot(ax_map, CVsyst_x, CVsyst_y, 'r+', 'MarkerSize', 10, 'LineWidth', 1.5);
hold(ax_map, 'off');
axis(ax_map, 'equal');
grid(ax_map, 'on');
xlim(ax_map, [-300, 300]);
ylim(ax_map, [-100, 700]);
xlabel(ax_map, 'X / mm');
ylabel(ax_map, 'Y / mm');
title(ax_map, 'Laser Mapping');

frame_count = 0;
valid_frame_count = 0;
loop_clock = tic;

%% Real-time test loop
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

        % mapping() includes the CV-system origin as its first point.
        if numel(map_x) > 1
            map_x = map_x(2:end);
            map_y = map_y(2:end);
        else
            map_x = [];
            map_y = [];
        end

        finite_points = isfinite(map_x) & isfinite(map_y);
        map_x = map_x(finite_points);
        map_y = map_y(finite_points);

        set(h_image, 'CData', image_rgb);
        set(h_mask, 'CData', laser_mask);
        set(h_map, 'XData', map_x, 'YData', map_y);

        valid_frame_count = valid_frame_count + 1;
        fps = valid_frame_count / max(toc(loop_clock), eps);
        title(ax_map, sprintf('Laser Mapping: %d points, %.2f FPS', ...
            numel(map_x), fps));

        if mod(frame_count, 10) == 0
            fprintf('[FRAME %d] laser_pixels=%d mapped_points=%d FPS=%.2f\n', ...
                frame_count, nnz(laser_mask), numel(map_x), fps);
        end

        drawnow limitrate;
    catch ME
        warning('[FRAME %d] %s', frame_count, ME.message);
        pause(0.1);
    end
end

clear camera_device;
fprintf('[STOP] Real laser mapping test stopped.\n');

%% Local functions
function mask = extractRealRedLaser(image_rgb, min_red_excess, ...
        min_red_value, min_hsv_saturation, min_hsv_value, ...
        min_component_area, max_component_area, ...
        min_component_eccentricity, min_component_major_axis)
%EXTRACTREALREDLASER Extract a thin red laser stripe from a real RGB image.

    hsv_image = rgb2hsv(image_rgb);
    hue = hsv_image(:,:,1);
    saturation = hsv_image(:,:,2);
    value = hsv_image(:,:,3);

    red = double(image_rgb(:,:,1));
    green = double(image_rgb(:,:,2));
    blue = double(image_rgb(:,:,3));

    % LaserFindRealtime-style HSV red extraction, with hue wrap-around
    % support for real red laser pixels close to either 0 or 1.
    hsv_red_mask = (hue >= 0.90 | hue <= 0.05) & ...
        saturation >= min_hsv_saturation & value >= min_hsv_value;

    % Extra RGB dominance gate suppresses bright white/orange background
    % pixels that may also pass a loose hue threshold.
    red_excess = red - max(green, blue);
    rgb_red_mask = red >= min_red_value & red_excess >= min_red_excess;

    mask = hsv_red_mask & rgb_red_mask;

    se = strel('disk', 2);
    mask = imdilate(mask, se);
    mask = imerode(mask, se);
    mask = bwareaopen(mask, min_component_area);

    components = bwconncomp(mask);
    stats = regionprops(components, 'Area', 'Eccentricity', 'MajorAxisLength');

    filtered = false(size(mask));
    for k = 1:numel(stats)
        is_reasonable_size = stats(k).Area >= min_component_area && ...
            stats(k).Area <= max_component_area;
        is_line_like = stats(k).Eccentricity >= min_component_eccentricity || ...
            stats(k).MajorAxisLength >= min_component_major_axis;

        if is_reasonable_size && is_line_like
            filtered(components.PixelIdxList{k}) = true;
        end
    end

    % Keep the full filtered laser region. Do not skeletonize here because
    % the real laser stripe can be weak or broken, and mapping can consume
    % all retained laser pixels directly.
    mask = filtered;
end
