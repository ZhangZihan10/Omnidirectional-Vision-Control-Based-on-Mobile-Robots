%%快速提取激光算法

function mask = extractRealRedLaser(image_rgb, min_red_excess, ...
        min_red_value, min_hsv_saturation, min_hsv_value, ...
        min_component_area, max_component_area, ...
        min_component_eccentricity, min_component_major_axis)
       %EXTRACTREALREDLASER 从真实 RGB 图像中提取细红色激光线。

    hsv_image  = rgb2hsv(image_rgb);
    hue        = hsv_image(:,:,1);
    saturation = hsv_image(:,:,2);
    value      = hsv_image(:,:,3);

    red   = double(image_rgb(:,:,1));
    green = double(image_rgb(:,:,2));
    blue  = double(image_rgb(:,:,3));

    hsv_red_mask = (hue >= 0.80 | hue <= 0.05) & ...
        saturation >= min_hsv_saturation & value >= min_hsv_value;

    red_excess  = red - max(green, blue);
    rgb_red_mask = red >= min_red_value & red_excess >= min_red_excess;

    %mask = hsv_red_mask & rgb_red_mask;
    mask = hsv_red_mask;

    se = strel('disk', 2);
    mask = imdilate(mask, se);
    mask = imerode(mask, se);
    mask = bwareaopen(mask, min_component_area);

    components = bwconncomp(mask);
    stats = regionprops(components, 'Area', 'Eccentricity', 'MajorAxisLength');

    filtered = false(size(mask));
    for k = 1:numel(stats)
        is_reasonable_size = stats(k).Area >= min_component_area && ...
            stats(k).Area <= max_component_area;
        is_line_like = stats(k).Eccentricity >= min_component_eccentricity || ...
            stats(k).MajorAxisLength >= min_component_major_axis;
        if is_reasonable_size && is_line_like
            filtered(components.PixelIdxList{k}) = true;
        end
    end

    mask = filtered;
end