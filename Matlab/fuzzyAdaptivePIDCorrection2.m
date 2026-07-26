function [pwm_delta, state] = fuzzyAdaptivePIDCorrection2(ref, actual, state, dt)
%FUZZYADAPTIVEPIDCORRECTION
% 输入轮速误差 e 和误差变化率 ec，通过模糊规则在线修正 Kp、Ki、Kd。
% 输出为 PWM 修正量，不是完整 PWM。

    e = ref - actual;
    ec = (e - state.e_prev) / dt;

    e_norm  = max(min(6 * e  / state.E_MAX,  6), -6);
    ec_norm = max(min(6 * ec / state.EC_MAX, 6), -6);

    [dKp, dKi, dKd] = fuzzyTunePID(e_norm, ec_norm, state);

    state.Kp = max(state.Kp0 + dKp, 0);
    state.Ki = max(state.Ki0 + dKi, 0);
    state.Kd = max(state.Kd0 + dKd, 0);

    state.integral = state.integral + e * dt;
    state.integral = max(min(state.integral, state.integral_limit), -state.integral_limit);

    derivative = (e - state.e_prev) / dt;

    pwm_delta = state.Kp * e + state.Ki * state.integral + state.Kd * derivative;
    pwm_delta = max(min(pwm_delta, state.output_limit), -state.output_limit);

    state.e_prev = e;
end


function [dKp, dKi, dKd] = fuzzyTunePID(e, ec, state)
%FUZZYTUNEPID
% 二维模糊控制器：
% 输入：e, ec
% 输出：dKp, dKi, dKd

    centers = [-6 -4 -2 0 2 4 6];

    mu_e  = triMembership7(e, centers);
    mu_ec = triMembership7(ec, centers);

    NB=-3; NM=-2; NS=-1; ZO=0; PS=1; PM=2; PB=3;

    rule_Kp = [
        PB PB PM PM PS PS ZO;
        PB PB PM PM PS ZO ZO;
        PM PM PM PS ZO NS NM;
        PM PS PS ZO NS NM NM;
        PS PS ZO NS NS NM NM;
        ZO ZO NS NM NM NM NB;
        ZO NS NS NM NM NB NB
    ];

    rule_Ki = [
        NB NB NB NM NM ZO ZO;
        NB NB NM NM NS ZO ZO;
        NM NM NS NS ZO PS PS;
        NM NS NS ZO PS PS PM;
        NS NS ZO PS PS PM PM;
        ZO ZO PS PM PM PB PB;
        ZO ZO PS PM PB PB PB
    ];

    rule_Kd = [
        PS PS ZO ZO ZO PB PB;
        NS NS NS NS ZO NS PM;
        NB NB NM NS ZO PS PM;
        NB NM NM NS ZO PS PM;
        NB NM NS NS ZO PS PS;
        NM NS NS NS ZO PS PS;
        PS ZO ZO ZO ZO PB PB
    ];

    out_Kp = fuzzyWeightedOutput(mu_e, mu_ec, rule_Kp);
    out_Ki = fuzzyWeightedOutput(mu_e, mu_ec, rule_Ki);
    out_Kd = fuzzyWeightedOutput(mu_e, mu_ec, rule_Kd);

    dKp = out_Kp / 3 * state.dKp_range;
    dKi = out_Ki / 3 * state.dKi_range;
    dKd = out_Kd / 3 * state.dKd_range;
end


function mu = triMembership7(x, centers)
% 7个三角隶属函数：NB NM NS ZO PS PM PB

    mu = zeros(1,7);
    width = 2;

    for i = 1:7
        mu(i) = max(0, 1 - abs(x - centers(i)) / width);
    end

    if x <= centers(1)
        mu(:) = 0;
        mu(1) = 1;
    elseif x >= centers(end)
        mu(:) = 0;
        mu(end) = 1;
    end
end


function y = fuzzyWeightedOutput(mu_e, mu_ec, rule_table)
% 加权平均清晰化

    numerator = 0;
    denominator = 0;

    for i = 1:7
        for j = 1:7
            w = mu_e(i) * mu_ec(j);
            numerator = numerator + w * rule_table(i,j);
            denominator = denominator + w;
        end
    end

    if denominator < 1e-9
        y = 0;
    else
        y = numerator / denominator;
    end
end