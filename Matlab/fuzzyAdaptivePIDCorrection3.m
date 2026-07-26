function [pwm_delta, state] = fuzzyAdaptivePIDCorrection3(ref, actual, state, dt)
%FUZZYADAPTIVEPIDCORRECTION3 Fast fuzzy-adaptive wheel-speed correction.
% The feedforward term is calculated by the caller. This function supplies
% a bounded feedback correction with derivative filtering and anti-windup.

    dt = max(dt, 1e-3);
    e = ref - actual;
    ec = (e - state.e_prev) / dt;

    % Clear stored integral when a wheel command genuinely reverses. This
    % prevents the previous direction's integral from fighting the turn.
    is_reversal = ref * state.ref_prev < 0 && ...
        abs(ref - state.ref_prev) > state.reversal_reset_threshold;
    if is_reversal
        state.integral = 0;
        state.derivative_filtered = 0;
    elseif abs(ref) < state.error_deadband
        state.integral = state.integral * state.integral_leak;
    end

    e_level = min(abs(e) / state.E_MAX, 1);
    ec_level = min(abs(ec) / state.EC_MAX, 1);
    error_growing = 0.5 * (1 + tanh(e * ec / 8e4));

    % Smooth fuzzy scheduling:
    % large/growing error -> stronger P; small error -> stronger I;
    % rapid changes/near-target motion -> stronger D.
    state.Kp = 0.022 + 0.026 * e_level + 0.008 * error_growing;
    state.Ki = 0.0010 + 0.0060 * (1 - e_level)^2;
    state.Kd = 0.0020 + 0.0050 * ec_level + 0.0025 * (1 - e_level);

    if abs(e) <= state.integral_enable_error
        integral_candidate = state.integral + e * dt;
    else
        integral_candidate = state.integral * state.integral_leak;
    end
    integral_candidate = max(min(integral_candidate, state.integral_limit), ...
        -state.integral_limit);

    % Derivative on measurement avoids a large kick when the speed target
    % changes at a new path segment.
    derivative_raw = -(actual - state.actual_prev) / dt;
    state.derivative_filtered = state.derivative_alpha * ...
        state.derivative_filtered + (1 - state.derivative_alpha) * derivative_raw;

    p_term = state.Kp * e;
    i_term = state.Ki * integral_candidate;
    d_term = state.Kd * state.derivative_filtered;
    output_unsat = p_term + i_term + d_term;

    % At low FPS, one correction is held for a long time. Reduce its
    % amplitude automatically to avoid overshoot around the feedforward.
    dt_scale = min(state.output_limit_reference_dt / dt, 1);
    active_output_limit = max(state.output_limit * dt_scale, ...
        state.output_limit_min);
    pwm_delta = max(min(output_unsat, active_output_limit), -active_output_limit);

    % Conditional integration: do not accumulate further into saturation.
    drives_further_into_limit = abs(output_unsat) > active_output_limit && ...
        sign(e) == sign(output_unsat);
    if ~drives_further_into_limit
        state.integral = integral_candidate;
    end

    if abs(e) < state.error_deadband
        pwm_delta = 0.35 * pwm_delta;
    end

    state.e_prev = e;
    state.actual_prev = actual;
    state.ref_prev = ref;
end
