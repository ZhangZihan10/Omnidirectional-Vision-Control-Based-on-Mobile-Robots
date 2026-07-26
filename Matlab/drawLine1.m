% 寻找中心线算法，   绘制线段的函数
function img = drawLine1(img, x1, y1, x2, y2)
    % Bresenham算法绘制线段
    % 确定绘制方向
    steep = abs(y2 - y1) > abs(x2 - x1);
    if steep
        [x1, y1] = swap(x1, y1);
        [x2, y2] = swap(x2, y2);
    end
    if x1 > x2
        [x1, x2] = swap(x1, x2);
        [y1, y2] = swap(y1, y2);
    end
    
    % 初始化变量
    dx = x2 - x1;
    dy = abs(y2 - y1);
    error = dx / 2;
    ystep = -1;
    if y1 < y2
        ystep = 1;
    end
    y = y1;
    
    % 绘制线段
    for x = x1:x2
        if steep
            img(x, y) = 1;
        else
            img(y, x) = 1;
        end
        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end
end