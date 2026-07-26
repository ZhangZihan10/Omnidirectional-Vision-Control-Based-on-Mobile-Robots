function pid_state = initFuzzyPIDState2(n)
%INITFUZZYPIDSTATE 初始化 n 路模糊自适应 PID 控制器状态

    template.e_prev = 0;
    template.integral = 0;

    template.Kp0 = 0.025;
    template.Ki0 = 0.003;
    template.Kd0 = 0.006;

    template.Kp = template.Kp0;
    template.Ki = template.Ki0;
    template.Kd = template.Kd0;

    template.E_MAX  = 500;
    template.EC_MAX = 300;

    template.dKp_range = 0.003;
    template.dKi_range = 0.0005;
    template.dKd_range = 0.001;

    template.integral_limit = 800;
    template.output_limit = 5;

    pid_state = repmat(template, n, 1);
end