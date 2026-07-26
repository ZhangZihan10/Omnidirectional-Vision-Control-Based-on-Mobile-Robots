function BW = las_segm1(img)
% 激光线图像分割函数（静默版，无图窗弹出）
% 输入: img - RGB 图像
% 输出: BW - 激光区域的二值掩膜图像

% 获取图像尺寸
%[height, width, ~] = size(img);

% 拆分 RGB 通道
r = img(:, :, 1);
g = img(:, :, 2);
b = img(:, :, 3);

% 阈值设定与蓝色区域判定（基于红色减去其它通道）
blueness = double(r) - max(double(g), double(b));
mask = blueness < 20;

% 构造彩色图像（非掩膜区域填白色）
R = r;
G = g;
B = b;
R(~mask) = 255;
G(~mask) = 255;
B(~mask) = 255;
J = cat(3, R, G, B); %#ok<NASGU> % 若需要输出彩色图像，可加额外输出参数

% 输出最终二值图像
BW = ~mask;

% 可选：开启骨架化
% BW = bwmorph(BW,'skel',Inf);

end