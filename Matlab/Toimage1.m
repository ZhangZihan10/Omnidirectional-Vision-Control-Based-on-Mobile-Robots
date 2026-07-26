function Toimage1(combinedVector1,combinedVector2)
x_min = -4000;
x_max = 4000;
y_min = -4000;
y_max = 4000;
ti=10;%缩小倍数
imageSize = [1000, 1000];  % 图像大小
image = zeros(imageSize);  % 创建全黑图像
validIndices = (combinedVector1 >x_min & combinedVector1 < x_max) & ...
               (combinedVector2 > y_min & combinedVector2 < y_max);
combinedVector1_filtered = combinedVector1(validIndices);
combinedVector2_filtered = combinedVector2(validIndices);
combinedVector1x =round(combinedVector1_filtered/ti)-round(x_min/ti);  % 第一组坐标点
combinedVector2y = round(combinedVector2_filtered/ti)-round(y_min/ti);  % 第二组坐标点

% 在坐标点处生成白色像素
for i = 1:length(combinedVector1x)
    x = combinedVector1x(i);
    y = 1000-combinedVector2y(i);
    image(y, x) = 255; % 将坐标点设置为白色
end
se = strel('disk', 1); % 定义圆形结构元素，半径为5个像素

% 对二值图像进行膨胀操作
dilatedImage = imdilate(image, se);
% 将图像保存为文件
imwrite(dilatedImage, 'output_image.png');


end