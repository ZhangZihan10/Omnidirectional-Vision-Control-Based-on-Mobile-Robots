% --- 找到最近节点 ---
function [idx, nearest] = find_nearest(tree, sample)
    distances = vecnorm(tree - sample, 2, 2);
    [~, idx] = min(distances);
    nearest = tree(idx, :);
end