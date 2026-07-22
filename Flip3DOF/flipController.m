function [T_cmd, tau_cmd] = flipController(s, ref, p)
% flipController  3DOF 평면 플립 제어기 (6DOF GeometricController의 평면판)
%
%   자세: 기하 PD + feedforward (sin 오차라 2pi 완주해도 안 꼬임)
%   고도: PD 제어 → 총추력 T
%   플립 페이즈: 플립 중엔 고도제어 OFF, 호버추력 고정 → 토크에 모터 예산 양보
%
%   s   = [x; z; theta; vx; vz; w]        상태
%   ref = struct(theta_des, w_des, thdd_des, z_des, inflip)   레퍼런스+페이즈
%   p   : m,J,g / kR,kW(자세) / kpz,kdz(고도)
%   반환: T_cmd(총추력 명령), tau_cmd(피치토크 명령)  → motorSat 통과 전 값

    theta = s(3);  z = s(2);  vz = s(5);  w = s(6);

    % --- 자세: 기하 PD + feedforward ---
    tau_cmd = p.J*ref.thdd_des ...
            - p.kR*sin(theta - ref.theta_des) ...
            - p.kW*(w - ref.w_des);

    % --- 추력: 페이즈에 따라 ---
    if ref.inflip
        T_cmd = p.m*p.g;                      % 플립 중: 고도제어 OFF (토크 양보)
    else
        az_des = p.kpz*(ref.z_des - z) + p.kdz*(0 - vz);   % 고도 PD (NED, +아래)
        ct = cos(theta);
        if abs(ct) < 0.3, ct = 0.3*sign(ct + eps); end     % 회전구간 특이점 방지
        T_cmd = p.m*(p.g - az_des)/ct;
    end
end
