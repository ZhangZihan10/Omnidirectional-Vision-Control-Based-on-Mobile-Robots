function path = rrt_star_path_planning2(start, goal, obstacle, safety_distance)
%RRT_STAR_PATH_PLANNING
% 改进版：增大扩展步长 + 路径剪枝，减少输出路径点数量

    max_iter = 1200;          % 最大迭代次数
    step_size = 1000;          % 扩展步长，原来 200，增大后路径点更少
    goal_tolerance = 500;     % 到达目标容忍距离
    neighbor_radius = 1500;    % 邻域半径，一般取 1.5*step_size 左右

    % 初始化
    tree = start;
    parent = -1;
    cost = 0;
    found = false;

    % 采样边界
    xmin = -8000;  xmax = 20000;
    ymin = -9000;  ymax = 20000;

    new_node = start;

    for i = 1:max_iter

        % 目标偏置
        if rand < 0.15
            sample = goal;
        else
            sample = [xmin, ymin] + rand(1,2) .* ([xmax - xmin, ymax - ymin]);
        end

        % 找最近节点
        [nearest_idx, nearest_node] = find_nearest(tree, sample);

        dir_vec = sample - nearest_node;
        dir_norm = norm(dir_vec);

        if dir_norm < 1e-6
            continue;
        end

        direction = dir_vec / dir_norm;
        new_node = nearest_node + step_size * direction;

        % 检查 nearest_node -> new_node 这一整段是否安全
        if check_segment_collision(nearest_node, new_node, obstacle, safety_distance)
            continue;
        end

        % 查找邻域节点
        neighbors = find_nearest_neighbors(tree, new_node, neighbor_radius);

        min_cost = cost(nearest_idx) + norm(nearest_node - new_node);
        best_parent = nearest_idx;

        for j = 1:length(neighbors)
            neighbor = neighbors(j);

            candidate_cost = cost(neighbor) + norm(tree(neighbor,:) - new_node);

            if candidate_cost < min_cost && ...
               ~check_segment_collision(tree(neighbor,:), new_node, obstacle, safety_distance)

                min_cost = candidate_cost;
                best_parent = neighbor;
            end
        end

        % 添加新节点
        tree = [tree; new_node]; %#ok<AGROW>
        parent = [parent; best_parent]; %#ok<AGROW>
        cost = [cost; min_cost]; %#ok<AGROW>

        new_idx = size(tree,1);

        % 尝试重连邻域节点，RRT* rewiring
        for j = 1:length(neighbors)
            neighbor = neighbors(j);

            new_cost = cost(new_idx) + norm(tree(neighbor,:) - new_node);

            if new_cost < cost(neighbor) && ...
               ~check_segment_collision(new_node, tree(neighbor,:), obstacle, safety_distance)

                parent(neighbor) = new_idx;
                cost(neighbor) = new_cost;
            end
        end

        % 判断是否可以直接连接目标
        if norm(new_node - goal) < goal_tolerance && ...
           ~check_segment_collision(new_node, goal, obstacle, safety_distance)

            tree = [tree; goal]; %#ok<AGROW>
            parent = [parent; new_idx]; %#ok<AGROW>
            cost = [cost; cost(new_idx) + norm(goal - new_node)]; %#ok<AGROW>
            found = true;
            break;
        end
    end

    % 回溯路径
    if found
        path = [];
        idx = size(tree, 1);

        while idx > 0
            path = [tree(idx,:); path]; %#ok<AGROW>
            idx = parent(idx);
        end

        % 路径剪枝，减少多余路径点
        path = pruneRRTPath(path, obstacle, safety_distance);

    else
        error('路径未找到');
    end
    
    function pruned_path = pruneRRTPath(path, obstacle, safety_distance)
%PRUNERRTPATH 删除 RRT* 路径中不必要的中间点

    if isempty(path) || size(path,1) <= 2
        pruned_path = path;
        return;
    end

    pruned_path = path(1,:);
    i = 1;

    while i < size(path,1)
        j = size(path,1);

        while j > i + 1
            if ~check_segment_collision(path(i,:), path(j,:), obstacle, safety_distance)
                break;
            end
            j = j - 1;
        end

        pruned_path = [pruned_path; path(j,:)]; %#ok<AGROW>
        i = j;
    end
    end

    function collision = check_segment_collision(p1, p2, obstacle, safety_distance)
%CHECK_SEGMENT_COLLISION 检查线段 p1-p2 是否与障碍物过近

    collision = false;

    if isempty(obstacle)
        return;
    end

    seg_len = norm(p2 - p1);

    if seg_len < 1e-6
        collision = check_collision(p1, obstacle, safety_distance);
        return;
    end

    sample_step = max(100, safety_distance / 3);
    n = max(2, ceil(seg_len / sample_step));

    for k = 0:n
        alpha = k / n;
        p = p1 + alpha * (p2 - p1);

        if check_collision(p, obstacle, safety_distance)
            collision = true;
            return;
        end
    end
end
    

    
end