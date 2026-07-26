function [pwm, state] = fuzzyAdaptivePIDController(ref, actual, state, dt)
%FUZZYADAPTIVEPIDCONTROLLER
% 单个电机的模糊自适应 PID 控制器
%
% 输入：
% ref    : 目标轮速 mm/s
% actual : 实际轮速 mm/s
% state  : PID 状态
% dt     : 控制周期
%
% 输出：
% pwm    : PWM 控制量，范围后续在主程序限制
% state  : 更新后的 PID 状态

    % ===== 1. 误差和误差变化率 =====
    e = ref - actual;
    ec = (e - state.e_prev) / dt;

    % ===== 2. 模糊自适应调整 PID 参数 =====
    [dKp, dKi, dKd] = fuzzyTunePID(e, ec);

    state.Kp = state.Kp0 + dKp;
    state.Ki = state.Ki0 + dKi;
    state.Kd = state.Kd0 + dKd;

    % 防止参数为负或过大
    state.Kp = max(min(state.Kp, 1.2), 0.01);
    state.Ki = max(min(state.Ki, 0.5), 0.00);
    state.Kd = max(min(state.Kd, 0.15), 0.00);

    % ===== 3. PID 控制 =====
    state.integral = state.integral + e * dt;

    % 防止积分饱和
    state.integral = max(min(state.integral, 500), -500);

    derivative = ec;

    pwm = state.Kp * e + state.Ki * state.integral + state.Kd * derivative;

    % ===== 4. 更新状态 =====
    state.e_prev = e;
end