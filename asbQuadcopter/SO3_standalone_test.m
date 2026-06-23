% SO3_dynamics + GeometricController standalone test
clear; clc;

Cbe = eye(3);
W   = zeros(3,1);
xN  = [57;95;-0.046];
vN  = zeros(3,1);

m  = 0.063;
J  = diag([5.82857e-5, 7.16914e-5, 1e-4]);
dt = 0.005;
g  = 9.81;
T_hover = -m*g;
kR = 0.012;  kW = 0.002;
R_des = eye(3);  W_des = zeros(3,1);

N = 300;  % 1.5s
t_log   = zeros(N,1);
DCM_log = zeros(N,9);
W_log   = zeros(N,3);
x_log   = zeros(N,3);
tau_log = zeros(N,2);
eR_log  = zeros(N,3);

for k = 1:N
    % GeometricController
    R_b2e = Cbe.';
    eR = vee3(0.5*(R_des.'*R_b2e - R_b2e.'*R_des));
    eW = W;
    tau = -kR*eR - kW*eW + cross(W, J*W);
    tau_pitch = tau(2);
    tau_roll  = tau(1);

    % Forces (body frame) - 처음 0.5초는 1.5x hover thrust로 이륙
    t_now = k * dt;
    if t_now < 0.5
        T_cmd = T_hover * 1.5;   % 이륙 thrust
    else
        T_cmd = T_hover;         % hover
    end
    F_grav   = Cbe * [0;0;m*g];
    F_thrust = [0;0; T_cmd];
    F_total  = F_thrust + F_grav;
    M = [tau_roll; tau_pitch; 0];

    % Translation
    aN = Cbe.' * (F_total./m);
    vN = vN + dt*aN;
    xN = xN + dt*vN;
    if xN(3) >= -0.046
        xN(3) = -0.046;
        if vN(3) > 0; vN(3)=0; end
    end

    % Rotation RK4
    k1 = J\(M - cross(W,          J*W));
    k2 = J\(M - cross(W+dt/2*k1, J*(W+dt/2*k1)));
    k3 = J\(M - cross(W+dt/2*k2, J*(W+dt/2*k2)));
    k4 = J\(M - cross(W+dt*k3,   J*(W+dt*k3)));
    Wmid = W + dt/2*k1;
    W    = W + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    Cbe  = expSO3m(-Wmid*dt)*Cbe;

    % normalize
    x = Cbe(:,1)/norm(Cbe(:,1));
    y = Cbe(:,2); y = y - x*(x'*y); y = y/norm(y);
    Cbe = [x, y, cross(x,y)];

    t_log(k)     = k*dt;
    DCM_log(k,:) = Cbe(:).';
    W_log(k,:)   = W.';
    x_log(k,:)   = xN.';
    tau_log(k,:) = [tau_pitch, tau_roll];
    eR_log(k,:)  = eR.';
end

fprintf('=== Standalone 1s 시뮬 결과 ===\n');
fprintf('최대 |W|:       %.6f rad/s\n', max(sqrt(sum(W_log.^2,2))));
fprintf('최대 |eR|:      %.6f\n',       max(sqrt(sum(eR_log.^2,2))));
fprintf('Cbe(1,1) 최종:  %.8f (기대≈1)\n', DCM_log(end,1));
fprintf('Cbe(2,2) 최종:  %.8f\n', DCM_log(end,5));
fprintf('Cbe(3,3) 최종:  %.8f\n', DCM_log(end,9));
fprintf('xN 최종:        [%.4f, %.4f, %.4f]\n', x_log(end,:));
fprintf('vN 최종:        [%.6f, %.6f, %.6f]\n', vN.');

function v = vee3(M)
    v = [M(3,2); M(1,3); M(2,1)];
end

function R = expSO3m(phi)
    th = norm(phi);
    K = [0,-phi(3),phi(2); phi(3),0,-phi(1); -phi(2),phi(1),0];
    if th < 1e-9
        R = eye(3) + K + 0.5*(K*K);
    else
        R = eye(3) + (sin(th)/th)*K + ((1-cos(th))/th^2)*(K*K);
    end
end
