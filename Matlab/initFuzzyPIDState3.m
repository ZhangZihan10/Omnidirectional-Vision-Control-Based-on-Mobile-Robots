function pid_state = initFuzzyPIDState3(n)
%INITFUZZYPIDSTATE3 Initialize the faster, filtered fuzzy-adaptive PID.

    template.e_prev = 0;
    template.actual_prev = 0;
    template.ref_prev = 0;
    template.integral = 0;
    template.derivative_filtered = 0;

    % Units are PWM per corresponding speed-error term.
    template.Kp = 0.030;
    template.Ki = 0.003;
    template.Kd = 0.003;

    template.E_MAX = 700;
    template.EC_MAX = 3500;
    template.integral_limit = 1100;
    template.output_limit = 28;
    template.output_limit_min = 9;
    template.output_limit_reference_dt = 0.20;
    template.integral_enable_error = 450;
    template.integral_leak = 0.92;
    template.derivative_alpha = 0.72;
    template.error_deadband = 8;
    template.reversal_reset_threshold = 200;

    pid_state = repmat(template, n, 1);
end
