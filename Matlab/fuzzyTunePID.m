function [dKp, dKi, dKd] = fuzzyTunePID(e, ec)
%FUZZYTUNEPID
% 简化模糊规则：
% e  大：增大 Kp，提高响应速度
% e  小：适当增大 Ki，消除稳态误差
% ec 大：增大 Kd，抑制振荡

    % 归一化，避免数值过大
    e_norm  = max(min(e  / 300,  1), -1);
    ec_norm = max(min(ec / 800,  1), -1);

    ae  = abs(e_norm);
    aec = abs(ec_norm);

    % ===== ΔKp：误差越大，Kp 越大 =====
    dKp = 0.45 * ae;

    % ===== ΔKi：误差较小时增强积分，误差大时减弱积分防止超调 =====
    dKi = 0.12 * (1 - ae);

    % ===== ΔKd：误差变化越剧烈，Kd 越大，抑制振荡 =====
    dKd = 0.06 * aec;

    % 如果误差和误差变化率方向相同，说明误差在扩大，应增强 Kp 和 Kd
    if e_norm * ec_norm > 0
        dKp = dKp + 0.10;
        dKd = dKd + 0.03;
    end

    % 如果误差很小，增强积分消除残余误差
    if ae < 0.15
        dKi = dKi + 0.08;
    end
end