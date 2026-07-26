% 参数设置
clear;
basePath = 'C:\论文\双鱼眼摄像头与机器人避障\图片\复原度测试/';      % 图像存放的文件夹路径
groupList = {'组18'};% {'组1', '组2','组3','组4', '组5','组6'};  % 实验组名称列表（根据实际组名修改）
numGroups = length(groupList); % 实验组数量

% 初始化存储结果的矩阵
hausdorffDist = zeros(numGroups, 1);
asd = zeros(numGroups, 1);
iou = zeros(numGroups, 1);

for i = 1:numGroups
    % --- 步骤1：读取原图和复原图 ---
    gtPath = fullfile(basePath, [groupList{i} '原.jpg']); % 原图路径
    recPath = fullfile(basePath, [groupList{i} '测.jpg']); % 复原图路径
    
    % 检查文件是否存在
    if ~exist(gtPath, 'file') || ~exist(recPath, 'file')
        error('文件缺失：请检查组 %s 的图像是否存在', groupList{i});
    end
    
    gtImage = imread(gtPath);   % 读取原图
    recImage = imread(recPath); % 读取复原图
    
    %figure;imshow(gtImage);figure;imshow(recImage);
    % --- 步骤2：预处理与轮廓提取（改进部分） ---
    % 转换为灰度图像
    gtGray = rgb2gray(gtImage);
    recGray = rgb2gray(recImage);
    
    % 使用局部自适应阈值（高斯加权窗口）
    windowSize = 39;       % 窗口大小（需为奇数，根据图像分辨率调整）
    sensitivity = 0.9;     % 敏感度（0~1，值越小阈值越严格）
    
    % 计算自适应阈值并二值化
    gtThresh = adaptthresh(gtGray, sensitivity, 'NeighborhoodSize', windowSize, 'Statistic', 'gaussian');
    gtBinary = imbinarize(gtGray, gtThresh);
    
    recThresh = adaptthresh(recGray, sensitivity, 'NeighborhoodSize', windowSize, 'Statistic', 'gaussian');
    recBinary = imbinarize(recGray, recThresh);
    % 后处理：形态学去噪
    se = strel('disk', 3);  % 创建圆形结构元素
    gtBinary = imopen(gtBinary, se); % 开运算去除小噪声
    gtBinary = ~gtBinary;
    figure;
    imshow(gtBinary);

    recBinary = imopen(recBinary, se);
    recBinary=~recBinary;
    figure;
    imshow(recBinary);
    
    
    
    % --- 步骤3：提取轮廓点集 ---
    [gtY, gtX] = find(edge(gtBinary, 'canny', [0.05 0.15])); % 原图轮廓
    [recY, recX] = find(edge(recBinary, 'canny', [0.05 0.15])); % 复原轮廓
    
    % 检查轮廓是否为空
    if isempty(gtX) || isempty(recX)
        warning('组 %s 的轮廓提取失败，跳过该组', groupList{i});
        continue;
    end

    % 检查轮廓是否为空
    if isempty(gtX) || isempty(recX)
        warning('组 %s 的轮廓提取失败，跳过该组', groupList{i});
        continue;
    end
    
    % --- 步骤4：轮廓配准（ICP对齐） ---
    %[transform, ~] = pcregistericp(...
    %    pointCloud([recX, recY, zeros(size(recX))]), ...
    %    pointCloud([gtX, gtY, zeros(size(gtX))]), ...
    %    'MaxIterations', 100 ...
    %);
    %recAligned = transform.T * [recX'; recY'; zeros(1, numel(recX)); ones(1, numel(recX))];
    %recX = recAligned(1,:)';
    %recY = recAligned(2,:)';

% --- 步骤5：计算指标 ---
    % Hausdorff距离
    hausdorffDist(i) = max(...
        max(pdist2([gtX, gtY], [recX, recY], 'euclidean'), [], 'all'), ...
        max(pdist2([recX, recY], [gtX, gtY], 'euclidean'), [], 'all') ...
    );
    
    % 平均对称距离（ASD）
    distGtToRec = min(pdist2([gtX, gtY], [recX, recY]), [], 2);
    distRecToGt = min(pdist2([recX, recY], [gtX, gtY]), [], 2);
    asd(i) = (mean(distGtToRec) + mean(distRecToGt)) / 2;
    
    % 交并比（IoU）
    [height, width] = size(gtBinary);
    maskGt = poly2mask(gtX, gtY, height, width);
    maskRec = poly2mask(recX, recY, height, width);
    intersection = maskGt & maskRec;
    union = maskGt | maskRec;
    iou(i) = sum(intersection(:)) / sum(union(:));
end

% --- 输出结果 ---
fprintf('实验组\tHausdorff距离\t平均对称距离\tIoU\n');
for i = 1:numGroups
    fprintf('%s\t%.2f\t\t%.2f\t\t%.2f\n', groupList{i}, hausdorffDist(i), asd(i), iou(i));
end
% --- 可选：保存结果到Excel ---
%resultTable = table(groupList', hausdorffDist, asd, iou, ...
%    'VariableNames', {'实验组', 'Hausdorff距离', '平均对称距离', 'IoU'});
%writetable(resultTable, fullfile(basePath, '复原精度结果.xlsx'));