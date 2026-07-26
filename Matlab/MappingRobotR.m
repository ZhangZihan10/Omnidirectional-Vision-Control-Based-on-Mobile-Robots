%真实系统，转化为机器人坐标系的目标物体进行绘图，并导出中心点坐标
function [potNew,x1,y1]=MappingRobotR(C_Up,Cube_l)

%转换全部图为机器人坐标系
C_Up_Transformed = zeros(size(C_Up));
% 遍历每一行进行转换
for i = 1:size(C_Up, 1)
    C_Up1 = C_Up(i, 1);  % 取出当前行的第一个元素
    C_Up2 = C_Up(i, 2);  % 取出当前行的第二个元素
    
    % 调用 PositionTran 函数进行转换
    % 假设 PositionTran 返回两个结果
    potNew1 = PositionTranR2(C_Up1, C_Up2);
    
    % 将转换后的结果存储到新的数组中
    C_Up_Transformed(i, :) = [potNew1(1), potNew1(2)];
end
% 提取 x 和 y 坐标
x = C_Up_Transformed(:, 1);  % 第一列为 x 坐标
y = C_Up_Transformed(:, 2);  % 第二列为 y 坐标

C_Up1_1 = mean(C_Up(:,1));%调用x的信息
C_Up2_1 = mean(C_Up(:,2));%调用y的信息
%C_Up1 = mean(C_Up(:,1));%调用x的信息
%C_Up2 = mean(C_Up(:,2));%调用y的信息
potNew2=PositionTranR2(C_Up1_1,C_Up2_1);%450为虚拟方形的边长一半
% 绘制 x 和 y 的散点图，点为蓝色
figure;
scatter(x,y,5,'filled'); % Laser intersections
hold on;
plot(potNew2(1),potNew2(2),'r*'); % CV System location
grid on;
hold off;
% 设置图形的标题和轴标签
title('Robot Coordinate System');
xlabel('X/cm');
ylabel('Y/cm');
potNew=C_Up_Transformed;
%potNew=potNew2;
x1=x;
y1=y;
end