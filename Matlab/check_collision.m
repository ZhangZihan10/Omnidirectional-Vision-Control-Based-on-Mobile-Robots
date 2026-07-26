% --- 碰撞检测函数 ---
function collision = check_collision(point, obstacle, safety_distance)
    % 计算点到障碍物轮廓的最小距离
    distances = sqrt(sum((obstacle - point).^2, 2)); % 每个轮廓点到节点的欧氏距离
    min_distance = min(distances); % 获取最小距离

    % 判断是否进入安全距离范围
    collision = min_distance <= safety_distance;
end