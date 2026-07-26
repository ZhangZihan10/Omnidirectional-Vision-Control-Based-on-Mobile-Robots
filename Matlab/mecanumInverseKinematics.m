function wheel_v = mecanumInverseKinematics(vx, vy, wz)
%MECANUMINVERSEKINEMATICS
% 车体速度 -> 四轮目标线速度
%
% 坐标约定：
% vx > 0：右移
% vy > 0：前进
% wz > 0：逆时针旋转
%
% 电机顺序按你之前小车代码：
% [0] ----- [1]
%  |         |
% [3] ----- [2]
%
% 这里输出顺序：
% wheel_v = [v0; v1; v2; v3]

    rot_gain = 120;  % 旋转速度折算系数，仿真中可调

    v0 = vy - vx + rot_gain * wz;
    v1 = vy + vx - rot_gain * wz;
    v2 = vy - vx - rot_gain * wz;
    v3 = vy + vx + rot_gain * wz;

    wheel_v = [v0; v1; v2; v3];
end