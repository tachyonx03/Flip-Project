%% flipTrajCheck.m  — 3DOF 평면 플립 드라이버 (6DOF asbQuadcopter 미러링)
%  구성: flipDynamics(플랜트) + flipController(제어기) + motorSat(액추에이터)
%  6DOF 미러 요소: 고도제어 / 플립 페이즈 추력스위치 / 이산제어(200Hz ZOH)
%  주의: 6DOF workspace 안 날리려고 clear 안 씀 (clc만).
clc;

% --- 파라미터 (asb Mambo) ---
p.m = 0.063;  p.J = 7.16914e-5;  p.g = 9.81;
p.l = 0.0441;  p.fmax = 0.3266;  p.fmin = 0.0065;   % 액추에이터
p.kR = 0.06;   p.kW = 0.006;                         % 자세 게인
p.kpz = 2.25;  p.kdz = 3.0;                          % 고도 게인

% --- 시나리오 ---
Ts   = 0.005;     % 제어 주기 (200Hz, 6DOF와 동일)
Tend = 10;
t0   = 6.0;       % 플립 시작 (3m 안정 후)
Tf   = 0.6;       % 한 바퀴 시간
zdes = -3;        % 목표 고도 3m (NED)

s = [0;0;0;0;0;0];            % [x z theta vx vz w], 지면(z=0) 시작
N = round(Tend/Ts);
LOG = zeros(N,8);             % t, z, theta, T_cmd, T, tau_cmd, tau, zref

for k = 1:N
    t = (k-1)*Ts;

    % --- 레퍼런스 + 페이즈 생성 ---
    ref.z_des  = zdes*min(t/4, 1);              % 4초 상승 후 hold
    ref.inflip = (t >= t0) && (t < t0+Tf);
    if t < t0
        ref.theta_des = 0;  ref.w_des = 0;  ref.thdd_des = 0;
    elseif ref.inflip
        x = (t-t0)/Tf;                          % 5차 min-jerk 0->2pi
        ref.theta_des = 2*pi*(10*x^3 - 15*x^4 + 6*x^5);
        ref.w_des     = 2*pi*(30*x^2 - 60*x^3 + 30*x^4)/Tf;
        ref.thdd_des  = 2*pi*(60*x - 180*x^2 + 120*x^3)/Tf^2;
    else
        ref.theta_des = 2*pi;  ref.w_des = 0;  ref.thdd_des = 0;
    end

    % --- 제어 (Ts마다 계산, ZOH) ---
    [T_cmd, tau_cmd] = flipController(s, ref, p);
    [T, tau] = motorSat(T_cmd, tau_cmd, p);     % 모터 한계: T,tau 경쟁

    LOG(k,:) = [t, s(2), s(3), T_cmd, T, tau_cmd, tau, ref.z_des];

    % --- 플랜트 적분 (RK4, 홀드된 T,tau로 Ts 전진) ---
    k1 = flipDynamics(0, s,         T, tau, p);
    k2 = flipDynamics(0, s+Ts/2*k1, T, tau, p);
    k3 = flipDynamics(0, s+Ts/2*k2, T, tau, p);
    k4 = flipDynamics(0, s+Ts*k3,   T, tau, p);
    s = s + Ts/6*(k1 + 2*k2 + 2*k3 + k4);
end

t=LOG(:,1); z=LOG(:,2); th=LOG(:,3);
Tc=LOG(:,4); Ta=LOG(:,5); tauc=LOG(:,6); taua=LOG(:,7); zr=LOG(:,8);

%% --- 플롯 (검은 배경 / 흰 선) ---
fig = figure('Name','3DOF flip (6DOF-mirrored)','Color','k');

subplot(2,2,1);
plot(t, th,'w','LineWidth',1.4); grid on; hold on;
yline(2*pi,'y--','2\pi'); xline(t0,'c:'); xline(t0+Tf,'c:');
xlabel('t [s]'); ylabel('\theta [rad]'); title('자세 \theta (2\pi=완주)');

subplot(2,2,2);
plot(t, -z,'w', t, -zr,'y--','LineWidth',1.4); grid on; hold on;
xline(t0,'c:'); xline(t0+Tf,'c:');
xlabel('t [s]'); ylabel('고도 -z [m]'); title('고도: 실제(흰) vs 목표(노랑)');
legend('actual','ref','Location','best');

subplot(2,2,3);
plot(t, Tc,'y--', t, Ta,'w','LineWidth',1.3); grid on;
xlabel('t [s]'); ylabel('T [N]'); title('총추력: 명령 vs 실제(포화)');
legend('cmd','sat','Location','best');

subplot(2,2,4);
plot(t, tauc,'y--', t, taua,'w','LineWidth',1.3); grid on;
xlabel('t [s]'); ylabel('\tau [N·m]'); title('토크: 명령 vs 실제(포화)');
legend('cmd','sat','Location','best');

ax=findobj(fig,'Type','axes');
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.8 .8 .8],'GridAlpha',.3);
set(findall(fig,'Type','text'),'Color','w');
set(findobj(fig,'Type','legend'),'TextColor','w','Color','k','EdgeColor','w');

%% --- 콘솔 판정 ---
iflip=find(t<=t0,1,'last');
fprintf('--- 3DOF flip (6DOF-mirrored) ---\n');
fprintf('플립직전 t=%.1fs 고도=%.2f m (목표 3)\n', t0, -z(iflip));
fprintf('theta 최대=%.2f, 최종=%.2f rad (2pi=%.2f)\n', max(th), th(end), 2*pi);
fprintf('플립 완주(2pi 도달)=%s\n', string(max(th)>=2*pi-0.1));
fprintf('최저 고도=%.2f m, 최종 고도=%.2f m (회복?)\n', min(-z), -z(end));
fprintf('토크 명령/실제 최대=%.4f / %.4f (한계~%.4f)\n', max(abs(tauc)), max(abs(taua)), 2*p.fmax*p.l);
