% --RRT* ------
function path = rrt_star_path_planning(start, goal, obstacle, safety_distance)
    max_iter = 1000;         % 最大迭代次数
    %step_size = 200;         % 扩展步长，单位：毫米
    %goal_tolerance = 100;    % 到达目标的容忍距离，单位：毫米
    %neighbor_radius = 300;   % 邻域半径，单位：毫米

    step_size = 600;
    goal_tolerance = 300;
    neighbor_radius = 900;

    % 初始化
    tree = start; 
    parent = -1;
    cost = 0; 
    found = false;

    % 采样边界（单位：毫米），设置为稍大于goal坐标范围
    xmin = -8000;  xmax = 20000;  % x 从 -2000 到 8000 毫米
    ymin = -9000;  ymax = 20000;  % y 从 -1000 到 9000 毫米


    for i = 1:max_iter
        if rand < 0.1
            sample = goal; % 增加目标偏好
        else
            sample = [xmin, ymin] + rand(1,2) .* ([xmax - xmin, ymax - ymin]);  % 随机采样范围扩大
        end

        % 找到最近节点
        [nearest_idx, nearest_node] = find_nearest(tree, sample);

        % 方向与新点
        direction = (sample - nearest_node) / norm(sample - nearest_node);
        new_node = nearest_node + step_size * direction;

        % 碰撞检测
        if ~check_collision(new_node, obstacle, safety_distance)
            % 查找邻域节点
            neighbors = find_nearest_neighbors(tree, new_node, neighbor_radius);
            min_cost = inf;
            best_parent = -1;

            for j = 1:length(neighbors)
                neighbor = neighbors(j);
                cost_to_new = cost(neighbor) + norm(tree(neighbor,:) - new_node);
                if ~check_collision(tree(neighbor,:), obstacle, safety_distance) && cost_to_new < min_cost
                    min_cost = cost_to_new;
                    best_parent = neighbor;
                end
            end

            % 添加新节点
            if best_parent > -1
                tree = [tree; new_node];
                parent = [parent; best_parent];
                cost = [cost; min_cost];
            end
        end

        % 判断是否接近目标
        if norm(new_node - goal) < goal_tolerance
            found = true;
            break;
        end
    end

    % 回溯路径
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
