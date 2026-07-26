function [vx, vy, wz] = mecanumForwardKinematics(wheel_v)
%MECANUMFORWARDKINEMATICS
% 四轮实际线速度 -> 小车实际车体速度

    v0 = wheel_v(1);
    v1 = wheel_v(2);
    v2 = wheel_v(3);
    v3 = wheel_v(4);

    vy = (v0 + v1 + v2 + v3) / 4;
    vx = (-v0 + v1 - v2 + v3) / 4;

    rot_gain = 120;
    wz = (v0 - v1 - v2 + v3) / (4 * rot_gain);
end