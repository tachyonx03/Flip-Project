%% flipTrajCheck.m
%  모터 한계(motorSat)를 반영한 클로즈드루프 플립.
%  제어기 = 평면 기하 PD + feedforward (asb GeometricController의 평면 축소판).
%    tau = J*thdd_des  - kR*sin(theta - theta_des) - kW*(w - w_des)
%          (feedforward)  (자세 P, sin기반)          (각속도 D)
%  자세각(theta=pitch)만 제어, 추력 T는 호버값 고정(고도제어는 다음 단계).
clear; clc;

% --- 파라미터 (asb Mambo) ---
p.m = 0.063;  p.J = 7.16914e-5;  p.g = 9.81;
p.l = 0.0441;  p.fmax = 0.3266;  p.fmin = 0.0065;   % 액추에이터
p.kR = 0.012;  p.kW = 0.002;                        % 기하 PD 게인 (asb 값)

t0 = 0.3;                     % 플립 시작 전 호버 시간
Tf = 0.35;                    % 플립(한바퀴) 목표 시간 [s]
tend = t0 + Tf + 0.7;

s0 = [0; -1; 0; 0; 0; 0];     % 고도 1m, 정지
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3);

[t, s] = ode45(@(t,s) plantCL(t,s,p,t0,Tf), [0 tend], s0, opts);

% --- 사후 재계산: 목표/명령·실제 토크 ---
N = numel(t);
[th_des, w_des, tau_cmd, tau_act] = deal(zeros(N,1));
for k = 1:N
    [th_des(k), w_des(k), thdd] = smoothFlip(t(k), t0, Tf);
    tc = p.J*thdd - p.kR*sin(s(k,3)-th_des(k)) - p.kW*(s(k,6)-w_des(k));
    tau_cmd(k) = tc;
    [~, tau_act(k)] = motorSat(p.m*p.g, tc, p);
end

%% --- 플롯 (검은 배경 / 흰 선) ---
fig = figure('Name','closed-loop flip (geometric PD)','Color','k');

subplot(2,2,1);
plot(t, s(:,3),'w', t, th_des,'y--','LineWidth',1.4); grid on;
yline(2*pi,'c:','2\pi'); xlabel('t [s]'); ylabel('\theta [rad]');
title('각도: 실제(흰) vs 목표(노랑)'); legend('actual','desired','Location','best');

subplot(2,2,2);
plot(t, s(:,6),'w', t, w_des,'y--','LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('\omega [rad/s]');
title('각속도: 실제 vs 목표'); legend('actual','desired','Location','best');

subplot(2,2,3);
plot(t, -s(:,2),'w','LineWidth',1.4); grid on;
yline(1,'c:','시작고도'); xlabel('t [s]'); ylabel('고도 -z [m]');
title('고도 (플립 중 낙하)');

subplot(2,2,4);
plot(t, tau_cmd,'y--', t, tau_act,'w','LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('\tau [N·m]');
title('토크: 명령(노랑) vs 실제(흰)'); legend('cmd','saturated','Location','best');

ax = findobj(fig,'Type','axes');
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.8 .8 .8],'GridAlpha',.3);
set(findall(fig,'Type','text'),'Color','w');
set(findobj(fig,'Type','legend'),'TextColor','w','Color','k','EdgeColor','w');

fprintf('--- 클로즈드루프 플립 (Tf=%.2fs, kR=%.3f kW=%.3f) ---\n', Tf, p.kR, p.kW);
fprintf('최종 theta = %.3f rad  (목표 2pi = %.3f, 오차 %.3f)\n', s(end,3), 2*pi, s(end,3)-2*pi);
fprintf('최종 각속도 = %.3f rad/s (0 기대)\n', s(end,6));
fprintf('최저 고도  = %.2f m   (시작 1.0 m)\n', min(-s(:,2)));

%% --- local functions ---
function ds = plantCL(t, s, p, t0, Tf)
    [th_d, w_d, thdd_d] = smoothFlip(t, t0, Tf);
    theta = s(3); vx = s(4); vz = s(5); w = s(6);
    % 평면 기하 PD + feedforward (sin 오차라 2pi 완주해도 안 꼬임)
    tau = p.J*thdd_d - p.kR*sin(theta - th_d) - p.kW*(w - w_d);
    T   = p.m*p.g;                       % 호버 추력 고정
    [T, tau] = motorSat(T, tau, p);      % 모터 한계 통과
    ax = -T*sin(theta)/p.m;
    az =  p.g - T*cos(theta)/p.m;
    ds = [vx; vz; w; ax; az; tau/p.J];
end

function [th, thd, thdd] = smoothFlip(t, t0, Tf)
% 5차 다항식(min-jerk) 궤적: 0 -> 2pi, 양 끝 속도/가속 0
    D = 2*pi;
    if t < t0
        th = 0;  thd = 0;  thdd = 0;
    elseif t < t0 + Tf
        s = (t - t0)/Tf;
        th   = D*(10*s^3 - 15*s^4 + 6*s^5);
        thd  = D*(30*s^2 - 60*s^3 + 30*s^4)/Tf;
        thdd = D*(60*s - 180*s^2 + 120*s^3)/Tf^2;
    else
        th = D;  thd = 0;  thdd = 0;
    end
end
