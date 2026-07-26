function flag = check_path_collision(path, obstacles, threshold)
    flag = false;
    for i = 1:size(path, 1)
        dist = sqrt((obstacles(:,1)-path(i,1)).^2 + (obstacles(:,2)-path(i,2)).^2);
        if any(dist < threshold)
            flag = true;
            return;
        end
    end
end
