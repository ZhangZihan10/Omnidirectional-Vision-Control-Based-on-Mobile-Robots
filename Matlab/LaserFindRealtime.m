function binaryImage = LaserFindRealtime(image)
% 实时激光线提取简化版，适用于循环处理
% 输入：RGB 图像
% 输出：激光线二值图

% 参数设定（红激光颜色范围，简化为一个掩膜）
hsvImage = rgb2hsv(image);
hue = 0.95; sat = 0.95; val = 0.95;
hue_offset = 0.05; sat_offset = 0.3; val_offset = 0.05;

lowerHSV = [hue - hue_offset, max(sat - sat_offset, 0), max(val - val_offset, 0)];
upperHSV = [1, 1, 1];

% 快速掩膜提取
mask = (hsvImage(:,:,1) >= lowerHSV(1)) & (hsvImage(:,:,1) <= upperHSV(1)) & ...
       (hsvImage(:,:,2) >= lowerHSV(2)) & (hsvImage(:,:,2) <= upperHSV(2)) & ...
       (hsvImage(:,:,3) >= lowerHSV(3)) & (hsvImage(:,:,3) <= upperHSV(3));

% 简单形态学处理
se = strel('disk', 2);
mask = imdilate(mask, se);
mask = imerode(mask, se);

% 骨架提取（仅用于细化）
skeleton = bwmorph(mask, 'skel', Inf);

% 可选：快速线段连接（不构建图结构）
% 提取坐标
[y, x] = find(skeleton);
points = [x, y];

% 若点太少，跳过
if numel(x) < 6
    binaryImage = false(size(image,1), size(image,2));
    return;
end

% 创建空白图像，用于画线
binaryImage = false(size(image,1), size(image,2));

% 快速连接线段（伪连接：相邻点近似视作直线）
D = pdist2(points, points);
[rowIdx, colIdx] = find(D < 50 & D > 0);
for i = 1:length(rowIdx)
    p1 = points(rowIdx(i), :);
    p2 = points(colIdx(i), :);
    binaryImage = insertLine(binaryImage, p1, p2);
end

% 转为逻辑图
binaryImage = binaryImage > 0;

end

function bw = insertLine(bw, p1, p2)
% 使用 Bresenham 算法快速插入线段到二值图像
lineCoords = round([linspace(p1(1), p2(1), 10); linspace(p1(2), p2(2), 10)]);
idx = sub2ind(size(bw), min(size(bw,1), max(1, lineCoords(2,:))), ...
                         min(size(bw,2), max(1, lineCoords(1,:))));
bw(idx) = 1;
end
