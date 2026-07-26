% 读取图像
cam1=webcam(3);
%preview(cam);

cam1.Resolution='1920x1080';
%cam.Brightness=-29;    %调整相机亮度

image =snapshot(cam1);
%image = imread('1.jpg'); % 替换为你的图像文件名

% 如果是彩色图像，转换为灰度图像
if size(image, 3) == 3
    image = rgb2gray(image);
end

% 获取图像的尺寸
[rows, cols] = size(image);

% 计算图像的中心点
centerX = cols / 2;
centerY = rows / 2;

% 在图像上标识中心点
markedImage = insertMarker(image, [centerX, centerY], 'o', 'Color', 'red', 'Size', 5);

% 显示原图像和标记后的图像
figure;
imshow(markedImage);
title('标记中心点后的图像');
