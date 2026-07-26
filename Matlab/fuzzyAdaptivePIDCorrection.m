function [pwm_delta, state] = fuzzyAdaptivePIDCorrection(ref, actual, state, dt)
%FUZZYADAPTIVEPIDCORRECTION
% 模糊自适应 PID 修正器
% 注意：这里输出的是 PWM 修正量，不是完整 PWM。

    e = ref - actual;
    ec = (e - state.e_prev) / dt;

    [dKp, dKi, dKd] = fuzzyTunePID(e, ec);

    state.Kp = state.Kp0 + dKp;
    state.Ki = state.Ki0 + dKi;
    state.Kd = state.Kd0 + dKd;

    % 参数限制，避免过大
    state.Kp = max(min(state.Kp, 0.35), 0.01);
    state.Ki = max(min(state.Ki, 0.08), 0.00);
    state.Kd = max(min(state.Kd, 0.03), 0.00);

    % 积分
    state.integral = state.integral + e * dt;
    state.integral = max(min(state.integral, 200), -200);

    derivative = ec;

    % 输出的是修正 PWM
    pwm_delta = state.Kp * e + state.Ki * state.integral + state.Kd * derivative;

    state.e_prev = e;
end