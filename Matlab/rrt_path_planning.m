% --- RRT 路径规划函数 ---
function path = rrt_path_planning(start, goal, obstacle,safety_distance)
    % RRT参数
    max_iter = 1000; % 最大迭代次数
    step_size = 0.01; % 扩展步长
    goal_tolerance = 0.01; % 到达目标的容忍距离

    % 初始化
    tree = start; % 树的起始节点
    parent = -1; % 每个节点的父节点索引
    found = false; % 是否找到路径

    % RRT主循环
    for i = 1:max_iter
        % 随机采样点
        if rand < 0.1
            sample = goal; % 增加目标偏好
        else
            sample = rand(1, 2) .* [0.3, 0.35]; % 随机点范围
        end

        % 找到距离最近的树节点
        [nearest_idx, nearest_node] = find_nearest(tree, sample);

        % 扩展树
        direction = (sample - nearest_node) / norm(sample - nearest_node);
        new_node = nearest_node + step_size * direction;

        % 碰撞检测
        if ~check_collision(new_node, obstacle,safety_distance)
            tree = [tree; new_node]; % 添加到树
            parent = [parent; nearest_idx]; % 记录父节点
        end

        % 检查是否到达目标
        if norm(new_node - goal) < goal_tolerance
            found = true;
            break;
        end
    end

    % 回溯生成路径
    if found
        path = goal;
        idx = size(tree, 1);
        while idx > 0
            path = [tree(idx, :); path];
            idx = parent(idx);
        end
    else
        error('路径未找到');
    end
end