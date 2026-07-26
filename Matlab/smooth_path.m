% --- 路径平滑化函数 ---
function smoothed_path = smooth_path(path)
    % 使用样条插值平滑路径
    t = 1:size(path, 1);
    t_interp = linspace(1, size(path, 1), 80);
    smoothed_path(:,1) = interp1(t, path(:,1), t_interp, 'spline');
    smoothed_path(:,2) = interp1(t, path(:,2), t_interp, 'spline');
end