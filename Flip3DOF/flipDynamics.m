function ds = flipDynamics(t, s, T, tau, p)
% s = [x; z; theta; vx; vz; w],  p.m / p.J / p.g
    theta = s(3);
    vx = s(4);  
    vz = s(5);  
    w = s(6);

    ax   = -T*sin(theta)/p.m;        % m·ẍ = -T sinθ
    az   =  p.g - (T/p.m)*cos(theta);% m·z̈ = mg - T cosθ
    wdot =  tau/p.J;                 % J·θ̈ = τ

    ds = [vx; vz; w; ax; az; wdot];
end

%[appendix]{"version":"1.0"}
%---
