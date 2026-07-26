function flag = check_path_collision3(path, obstacles, threshold)
%CHECK_PATH_COLLISION3 Check obstacle distance to every remaining path segment.

    flag = false;
    if isempty(path) || isempty(obstacles)
        return;
    end

    if size(path, 1) == 1
        flag = any(vecnorm(obstacles - path(1,:), 2, 2) < threshold);
        return;
    end

    threshold_sq = threshold^2;
    for i = 1:size(path, 1)-1
        p0 = path(i,:);
        segment = path(i+1,:) - p0;
        segment_length_sq = dot(segment, segment);

        if segment_length_sq < eps
            distance_sq = sum((obstacles - p0).^2, 2);
        else
            projection = ((obstacles - p0) * segment') / segment_length_sq;
            projection = max(min(projection, 1), 0);
            nearest = p0 + projection .* segment;
            distance_sq = sum((obstacles - nearest).^2, 2);
        end

        if any(distance_sq < threshold_sq)
            flag = true;
            return;
        end
    end
end
