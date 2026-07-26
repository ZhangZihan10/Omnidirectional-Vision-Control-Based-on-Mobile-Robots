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




% ======= 输入数据：两个点在两个坐标系中的坐标 =======
% D1 点
xD1w = 0.54*10;
yD1w = -0.8462*10;
xD1c = -3.3*10;  %虚拟环境中，在cv系统内的定位数据/1000
yD1c = -4.998*10;

% D2 点
xD2w = 0.54*10;
yD2w = 1.32*10;
xD2c = -1.1537*10;
yD2c = -4.988*10;

% ======= 第一步：构造两个向量 =======
vw = [xD2w - xD1w; yD2w - yD1w];  % 世界系向量
vc = [xD2c - xD1c; yD2c - yD1c];  % 相机系向量

% ======= 第二步：计算旋转角度 =======
theta = atan2(vw(2), vw(1)) - atan2(vc(2), vc(1));

% ======= 第三步：构造旋转矩阵 =======
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];

% ======= 第四步：计算平移向量（相机原点在世界坐标系的位置） =======
T = [xD1w; yD1w] - R * [xD1c; yD1c];
xc = T(1);
yc = T(2);

% 输出结果
fprintf('相机原点在世界坐标系中的位置: (%.4f, %.4f)\n', xc, yc);
fprintf('相机相对于世界坐标系的旋转角度: %.4f rad (%.2f deg)\n', ...
        theta, rad2deg(theta));

% ======= 可视化部分 =======
figure; hold on; axis equal;
title('世界坐标系下的位置与变换可视化');
xlabel('X/cm'); ylabel('Y/cm');
grid on;

% 绘制 D1 和 D2 在世界坐标系下的位置
plot(xD1w, yD1w, 'ro', 'MarkerSize', 8, 'DisplayName', 'D1 (World)');
text(xD1w + 2, yD1w + 2, 'D1_w');

plot(xD2w, yD2w, 'bo', 'MarkerSize', 8, 'DisplayName', 'D2 (World)');
text(xD2w + 2, yD2w + 2, 'D2_w');

% 绘制相机原点在世界坐标系下的位置
plot(xc, yc, 'ks', 'MarkerSize', 10, 'DisplayName', 'Camera Origin');
text(xc + 2, yc + 2, 'Camera Origin');

% 绘制相机坐标系的轴（在世界坐标系下）
axis_len = 15;  % 坐标轴长度

% X 轴方向（红色）
cam_x_axis = R * [axis_len; 0];
quiver(xc, yc, cam_x_axis(1), cam_x_axis(2), 0, 'r', 'LineWidth', 2, 'DisplayName', 'Camera X');

% Y 轴方向（绿色）
cam_y_axis = R * [0; axis_len];
quiver(xc, yc, cam_y_axis(1), cam_y_axis(2), 0, 'g', 'LineWidth', 2, 'DisplayName', 'Camera Y');

legend('Location', 'best');
