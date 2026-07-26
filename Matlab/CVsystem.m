%平均值算法
function output=CVsystem(a)
% Laser Segmentation
%name = "Matlab";
%Client = TCPInit('127.0.0.1',55012,name);
Client=a;
image =ImageReadTCP_One(Client,'Center'); %imread('TestImages/image6.jpg');
image1 = ImageReadTCP_One1(Client,'Center');%imread('TestImages/image7.jpg');
img = las_segm(image);
img1 = las_segm(image1);
% Configuration
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
camX =0;%-2.5; % Camera parameters
camY =0;%6; % Camera parameters
camZ =0;% 3; % Camera parameters
lasX = 0;%1.5; % Laser Plane parameters
lasY = 0;%-2.5; % Laser Plane parameters
las_dist = 950; % Laser Plane parameters
CVsyst_x = -500; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y =-2000; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
CVsyst_rot = 0; % CV System initial rotation
CVsyst_x1 = 2000; % CV System second position 在unity中为CVSystemOrigin2的位置参数z*1000
CVsyst_y1 = 4000; % CV System second position 在unity中为CVSystemOrigin2的位置参数x*-1000
CVsyst_rot1 = 0;%20; % CV System second rotation 在unity中为CVSystemOrigin2的位置参数Rotation Y
% Mapping
[x,y] = mapping(img,CVsyst_rot,CVsyst_x,CVsyst_y,camX,camY,camZ,lasX,lasY,...
    las_dist,ocam_model); % mapping function
[x1,y1] = mapping(img1,CVsyst_rot1,CVsyst_x1,CVsyst_y1,camX,camY,camZ,lasX,...
    lasY,las_dist,ocam_model); % mapping function
% Finally figure:
figure;
scatter(x,y,5,'filled'); % Laser intersections, first image
hold on;
plot(CVsyst_x,CVsyst_y,'r*'); % CV System location, first image
scatter(x1,y1,5,'filled'); % Laser intersections, second image
plot(CVsyst_x1,CVsyst_y1,'r*'); % CV System location, second image
grid on;
combinedVector1 = [x, x1];
combinedVector2 = [y, y1];
combinedVector1 = nonzeros(combinedVector1);
combinedVector2 = nonzeros(combinedVector2);
x2=mean(combinedVector1);
y2=mean(combinedVector2);
%x2=(max(combinedVector1)+min(combinedVector1))/2;
%y2=(max(combinedVector2)+min(combinedVector2))/2;
output=[x2,y2];
end