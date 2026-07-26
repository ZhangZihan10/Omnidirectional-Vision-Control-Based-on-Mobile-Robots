function motor_speed = simulateTTMotor(motor_speed, pwm_cmd, dt, tau, gain)
%SIMULATETTMOTOR
% TT 减速直流电机一阶模型，带速度限幅

    MAX_REAL_WHEEL_SPEED = 600;   % 仿真最大实际轮速 mm/s

    target_speed = gain * pwm_cmd;

    motor_speed = motor_speed + ...
        (target_speed - motor_speed) * dt / tau;

    motor_speed = max(min(motor_speed, MAX_REAL_WHEEL_SPEED), -MAX_REAL_WHEEL_SPEED);
end