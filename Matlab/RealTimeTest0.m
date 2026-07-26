% 实时循环示例
Client = TCPInit('127.0.0.1',55012,'Realtime');

% 设置起点和终点
startPoint = [0, 0];
endPoint = [5000, -4000];

% 设定路径点数（可根据需要调整分辨率）
numPoints = 100;

% 生成线性插值路径
x = linspace(startPoint(1), endPoint(1), numPoints);
y = linspace(startPoint(2), endPoint(2), numPoints);

% 组合成 q 路径矩阵（每行是一个 [x, y] 坐标点）
q = [x', y'];

% 可视化轨迹
figure;
plot(q(:,1), q(:,2), 'b.-');
xlabel('X');
ylabel('Y');
title('机器人运动轨迹');
grid on;

%robot.plot(q);传输到unity
b = 1;
for a = 1 : length(q)
    func_Car(Client, q, b);
    b=b+1;     
end

while true
    frame = ImageReadTCP_One(Client, 'Center');
    laser = LaserFindRealtime(frame);
    subplot(1,2,1);
    imshow(frame);
    subplot(1,2,2);
    imshow(laser);
    drawnow;
end
