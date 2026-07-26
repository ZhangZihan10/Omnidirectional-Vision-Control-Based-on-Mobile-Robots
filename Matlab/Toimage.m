function Toimage(combinedVector1,combinedVector2)
x_min = min(combinedVector1);
%x_max = max(combinedVector1);
y_min = min(combinedVector2);
%y_max = max(combinedVector2);
imageSize = [1000, 1000];  % 图像大小
image = zeros(imageSize);  % 创建全黑图像
combinedVector1x =round(combinedVector1/2)-round(x_min/2)+10;  % 第一组坐标点
combinedVector2y = round(combinedVector2/2)-round(y_min/2)+10;  % 第二组坐标点

% 在坐标点处生成白色像素
for i = 1:length(combinedVector1x)
    x = combinedVector1x(i);
    y = 1000-combinedVector2y(i);
    image(y, x) = 255; % 将坐标点设置为白色
end
se = strel('disk', 5); % 定义圆形结构元素，半径为5个像素

% 对二值图像进行膨胀操作
dilatedImage = imdilate(image, se);
% 将图像保存为文件
imwrite(dilatedImage, 'output_image.png');


end