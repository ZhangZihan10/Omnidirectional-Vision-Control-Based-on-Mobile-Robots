% ======= 输入：两点在世界与相机坐标系下的坐标 =======
clc; clear;
%虚拟系统，平均值算法计算目标中心
% Laser Segmentation
name = "Matlab";
%a=Client;
Client = TCPInit('127.0.0.1',55014,name);
%Client=a;
image1 = ImageReadTCP_One1(Client,'Center');%imread('TestImages/image7.jpg');
%img = las_segm(image);
%img1 = las_segm(image1);
img1 = LaserFind(image1);
% Configuration
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters


camX1 =0;%-2.5; % Camera parameters 为z旋转
camY1 = 0;%6; % Camera parameters 为x旋转
camZ1 =0;% 3; % Camera parameters 为y旋转
lasX1 = 0;%1.5; % Laser Plane parameters
lasY1 = 0;%-2.5; % Laser Plane parameters
las_dist1 = 950; % Laser Plane parameters


%CVsyst_rot = 0; % CV System initial rotation
CVsyst_x1 = 0; % CV System second position 在unity中为CVSystemOrigin2的位置参数z*1000
CVsyst_y1 = 0; % CV System second position 在unity中为CVSystemOrigin2的位置参数x*-1000
CVsyst_rot1 = 0;%20; % CV System second rotation 在unity中为CVSystemOrigin2的位置参数Rotation Y
% Mapping

[x1,y1] = mapping(img1,CVsyst_rot1,CVsyst_x1,CVsyst_y1,camX1,camY1,camZ1,lasX1,...
    lasY1,las_dist1,ocam_model); % mapping function
% Finally figure:
figure;
scatter(x1,y1,5,'filled'); % Laser intersections, second image
%plot(CVsyst_x1,CVsyst_y1,'r*'); % CV System location, second image
grid on;



% ====== 输入2个点的坐标（单位：cm） ======
% 世界坐标系下坐标 [x, y]
Pw = [ 0.54,  -0.8462;
       0.54,   1.32; %] * 10;
       -0.8,   0.5 ] * 10;

% 相机坐标系下坐标（虚拟环境中，乘以10变为cm）
Pc = [-3.3,   -4.998;
      -1.1537, -4.988; %] * 10;
      -1.955, -3.6239] * 10;

% 转置成 2xN 矩阵（方便矩阵运算）
Pw = Pw';  % 2xN
Pc = Pc';  % 2xN

Pwt=[-0.5,  -1;   -1.1,  0.8]* 10;
Pct=[-3.5729,-1.687; -3.9728,-3.474]* 10;

% ====== 求刚体变换（最小二乘拟合） ======

% 计算质心
cw = mean(Pw, 2);  % 世界坐标系质心
cc = mean(Pc, 2);  % 相机坐标系质心

% 去中心化
Pw0 = Pw - cw;
Pc0 = Pc - cc;

% 计算旋转矩阵 R （使用 SVD 解）
H = Pc0 * Pw0';
[U, ~, V] = svd(H);
R = V * U';

% 修正反转的情况（防止 det(R) < 0）
if det(R) < 0
    V(:, end) = -V(:, end);
    R = V * U';
end

% 平移向量
T = cw - R * cc;

% 输出相机原点在世界坐标系位置
xc = T(1);
yc = T(2);

% 旋转角度（从相机 → 世界）
theta = atan2(R(2,1), R(1,1));

fprintf('相机原点在世界坐标系的位置: (%.4f, %.4f)\n', xc, yc);
fprintf('旋转角度 θ: %.4f rad (%.2f deg)\n', theta, rad2deg(theta));
%fprintf('R %.4f\n', R);
%fprintf('T %.4f\n', T);

% ====== 计算投影误差 ======
num_points = size(Pwt, 2);
errors = zeros(num_points, 1);

for i = 1:num_points
    Pc_proj = R * Pct(:, i) + T;
    errors(i) = norm(Pc_proj - Pwt(:, i));
    fprintf('点 %d 的投影误差: %.4f cm\n', i, errors(i));
end

% ====== 可视化 ======
figure; hold on; axis equal;
title('世界坐标系下的相机坐标系拟合');
xlabel('X/cm'); ylabel('Y/cm');
grid on;

% 绘制世界坐标系下原始点
plot(Pw(1, :), Pw(2, :), 'bo', 'MarkerSize', 8, 'DisplayName', 'World Points');

% 绘制重建的点
for i = 1:num_points
    Pc_proj = R * Pc(:, i) + T;
    plot(Pc_proj(1), Pc_proj(2), 'rx', 'MarkerSize', 10);
    line([Pc_proj(1), Pw(1, i)], [Pc_proj(2), Pw(2, i)], 'Color', 'k', 'LineStyle', '--');
end

% 绘制相机原点及坐标轴
plot(xc, yc, 'ks', 'MarkerSize', 10, 'DisplayName', 'Camera Origin');
text(xc + 2, yc + 2, 'Camera Origin');

axis_len = 15;
qx = R * [axis_len; 0];
qy = R * [0; axis_len];
quiver(xc, yc, qx(1), qx(2), 0, 'r', 'LineWidth', 2, 'DisplayName', 'Camera X');
quiver(xc, yc, qy(1), qy(2), 0, 'g', 'LineWidth', 2, 'DisplayName', 'Camera Y');

legend('Location', 'best');

% 平均投影误差
mean_error = mean(errors);
rmse = sqrt(mean(errors.^2));

fprintf('平均投影误差: %.4f cm\n', mean_error);
fprintf('RMSE: %.4f cm\n', rmse);

