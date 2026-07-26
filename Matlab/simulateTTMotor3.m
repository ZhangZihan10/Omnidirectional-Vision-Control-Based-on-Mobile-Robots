function motor_speed = simulateTTMotor3(motor_speed, pwm_cmd, dt, tau, gain, max_speed)
%SIMULATETTMOTOR3 First-order TT motor model consistent with commanded speed.

    target_speed = gain * pwm_cmd;
    response_ratio = 1 - exp(-dt / max(tau, 1e-3));
    motor_speed = motor_speed + (target_speed - motor_speed) * response_ratio;
    motor_speed = max(min(motor_speed, max_speed), -max_speed);
end
