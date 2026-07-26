function pid_state = initFuzzyPIDState(n)
%INITFUZZYPIDSTATE 初始化 n 路模糊自适应 PID 控制器状态

    template.e_prev = 0;
    template.integral = 0;

    % 由于现在 PID 只做修正量，参数要小
    template.Kp0 = 0.08;
    template.Ki0 = 0.01;
    template.Kd0 = 0.002;

    template.Kp = template.Kp0;
    template.Ki = template.Ki0;
    template.Kd = template.Kd0;

    pid_state = repmat(template, n, 1);
end