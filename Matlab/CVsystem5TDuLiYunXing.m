%真实系统，平均值算法计算目标中心

% Laser Segmentation
%name = "Matlab";
%Client = TCPInit('127.0.0.1',55012,name);
Cub_l=10;%木块的边长一半单位mm
%cam=webcam(3);cam.Resolution='1920x1080';
%preview(cam);
%cam.Brightness=-30;%调整相机亮度%cam.WhiteBalanceMode='manu'

%cam1=webcam(2);cam1.Resolution='1920x1080';
%preview(cam1);
%cam1.Brightness=-30;%调整相机亮度
%cam1.Exposure=-7;%cam1.WhiteBalanceMode='manu'%cam1.Sharpness=1;

image =snapshot(cam);
image1 =snapshot(cam1);

%img1 = las_segm(image);
img=LaserFind3(image);% LaserFind使用霍夫变换寻找直线，LaserFind2
img1 = LaserFind3(image1);

% Mapping
load('Omni_Calib_Results1.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters

camX =0;%-2.5; % Camera parameters
camY =0;%6; % Camera parameters
camZ =0;% 3; % Camera parameters
lasX = 0;%1.5; % Laser Plane parameters
lasY = 0;%-2.5; % Laser Plane parameters
las_dist = 131; % Laser Plane parameters
CVsyst_x = 0; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y =0; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
CVsyst_rot = 0; % CV System initial rotation
CVsyst_x1 = 102; % CV System second position 在unity中为CVSystemOrigin2的位置参数z*1000
CVsyst_y1 = 252; % CV System second position 在unity中为CVSystemOrigin2的位置参数x*-1000
CVsyst_rot1 = -90;%20; % CV System second rotation 在unity中为CVSystemOrigin2的位置参数Rotation Y
las_dist1 = 137; % Laser Plane parameters
% Mapping
[x,y] = mapping(img,CVsyst_rot,CVsyst_x,CVsyst_y,camX,camY,camZ,lasX,lasY,...
    las_dist,ocam_model); % mapping function
[x1,y1] = mapping(img1,CVsyst_rot1,CVsyst_x1,CVsyst_y1,camX,camY,camZ,lasX,...
    lasY,las_dist1,ocam_model); % mapping function
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

combinedVector1(combinedVector1==CVsyst_x1)=[];
combinedVector2(combinedVector2==CVsyst_y1)=[];%去除向量内的原点坐标，减少干扰
combinedVector1(combinedVector1==CVsyst_x)=[];
combinedVector2(combinedVector2==CVsyst_y)=[];%去除向量内的原点坐标，减少干扰

%将激光矫正数据还原成图片
Toimage1(combinedVector1,combinedVector2);

%x2=mean(combinedVector1);
%y2=mean(combinedVector2);

%x2=(max(combinedVector1)+min(combinedVector1))/2;
%y2=(max(combinedVector2)+min(combinedVector2))/2;

%寻找偏移角度
[Y1max,Y1maxRow]=max(combinedVector2);%寻找y最大值
X1value = combinedVector1(Y1maxRow);
[X3max,X3Row]=max(combinedVector1);%寻找x最大值
Y3value = combinedVector2(X3Row);
[X2min,X2Row]=min(combinedVector1);%寻找x最小值
Y2value = combinedVector2(X2Row);
[Y4min,Y4minRow]=min(combinedVector2);%寻找y最小值
X4value = combinedVector1(Y4minRow);
%寻找立方体中心点
L1=sqrt((X1value-X4value)^2+(Y1max-Y4min)^2);
L2=sqrt((X1value-X3max)^2+(Y1max-Y3value)^2);
L3=sqrt((X3max-X4value)^2+(Y3value-Y4min)^2);
thetha1=acosd((L1^2+L2^2-L3^2)/(2*L1*L2));
thetha2=thetha1-atan2d(X3max-X1value,Y1max-Y3value);%立方体中心点与y1点的偏角

test1=y1(100)-Y1max;%判断物体是否有偏移
if abs(test1)<50
    thetha6=0;
    x2=mean(combinedVector1);
    y2=mean(combinedVector2);
else
   thetha6=90-atan2d(X3max-X1value,Y1max-Y3value);
   x2=X1value-L1/2*sind(thetha2);
   y2=Y1max-L1/2*cosd(thetha2);

end

%绘制机器人坐标系图像
%POutput=[combinedVector1,combinedVector2];
%[potOld]=MappingRobotV(POutput);

POutput=[x;y];
POutput=reshape(POutput', [], 2);
[potOld]=MappingRobotR(POutput);

POutput1=[x1,y1];
POutput1=reshape(POutput1', [], 2);
[potOld1]=MappingRobotR(POutput1);

%相机坐标转换
CVsyst=PositionTranR2(CVsyst_x,CVsyst_y);
CVsyst1=PositionTranR2(CVsyst_x1,CVsyst_y1);

% 合并两个数据变量
mergedData = [potOld; potOld1];

% 提取满足条件的数据 (10 < x < 16)
filteredData = mergedData(mergedData(:,1) > 10 & mergedData(:,1) < 16 & ...
                          mergedData(:,2) > -10 & mergedData(:,2) < 10, :);
% 计算 x 和 y 的平均值
averageX = mean(filteredData(:,1));
averageY = mean(filteredData(:,2));



% Finally figure:
figure;
scatter(potOld(:,1),potOld(:,2),5,'filled'); % Laser intersections, first image
hold on;
plot(CVsyst(1),CVsyst(2),'r*'); % CV System location, first image
scatter(potOld1(:,1),potOld1(:,2),5,'filled'); % Laser intersections, second image
plot(CVsyst1(1),CVsyst1(2),'r*'); % CV System location, second image
plot(averageX,averageY,'r*');
grid on;



output=[x2,y2,thetha6];
