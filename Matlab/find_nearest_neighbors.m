function neighbors = find_nearest_neighbors(tree, new_node, radius)
    distances = vecnorm(tree - new_node, 2, 2);
    neighbors = find(distances < radius); % 找到邻域内的所有节点
end