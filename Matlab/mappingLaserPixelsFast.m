function [x, y] = mappingLaserPixelsFast(image, cvsyst_rot, cvsyst_y, ...
        cvsyst_x, camY, camX, camZ, lasY, lasX, las_dist, ocam_model)
%MAPPINGLASERPIXELSFAST Vectorized mapping for nonzero laser-mask pixels.
% The output layout matches mapping(): the first point is the CV origin.

    [rows, cols] = find(image > 0);
    if isempty(rows)
        x = [];
        y = [];
        return;
    end

    zLaser = las_dist;
    laserRotation = compose_rotation(-lasX, -lasY, 0);
    cameraRotation = compose_rotation(camX, camY, camZ);
    transform = cameraRotation * [laserRotation(:,1), ...
        laserRotation(:,2), [0; 0; zLaser]];

    rays = cam2world([rows.'; cols.'], ocam_model);
    a1 = rays(1,:) * transform(2,1) - rays(2,:) * transform(1,1);
    b1 = rays(1,:) * transform(2,2) - rays(2,:) * transform(1,2);
    c1 = rays(1,:) * transform(2,3) - rays(2,:) * transform(1,3);
    a2 = rays(3,:) * transform(1,1) - rays(1,:) * transform(3,1);
    b2 = rays(3,:) * transform(1,2) - rays(1,:) * transform(3,2);
    c2 = rays(3,:) * transform(1,3) - rays(1,:) * transform(3,3);

    denominator = a1 .* b2 - a2 .* b1;
    valid = abs(denominator) > 1e-9 & abs(a1) > 1e-9;
    planeY = nan(size(denominator));
    planeX = nan(size(denominator));
    planeY(valid) = (a2(valid) .* c1(valid) - ...
        a1(valid) .* c2(valid)) ./ denominator(valid);
    planeX(valid) = (-c1(valid) - b1(valid) .* planeY(valid)) ./ a1(valid);

    cvRotation = compose_rotation(0, 0, -cvsyst_rot);
    worldPoints = cvRotation * [planeX; planeY; ones(size(planeX))];

    x = [cvsyst_y, -worldPoints(2,:) + cvsyst_y];
    y = [cvsyst_x,  worldPoints(1,:) + cvsyst_x];
end
