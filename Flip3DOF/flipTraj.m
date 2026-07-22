
function [th_des, w_des] = flipTraj(t, T_flip)
% 직선(일정속도) 백플립 안무
%   t      : 현재 시각 [s]
%   T_flip : 한 바퀴 도는 데 걸리는 시간 [s]
%   th_des : 목표 각도 [rad]      (0 → 2π)
%   w_des  : 목표 각속도 [rad/s]  (제어기에서 쓸 예정)

    if t < T_flip
        th_des = 2*pi * t / T_flip;   % 시간 비례해서 0 → 2π 직선
        w_des  = 2*pi / T_flip;       % 일정한 각속도
    else
        th_des = 2*pi;                % 한 바퀴 다 돌고
        w_des  = 0;                   % 멈춤
    end
end

%[appendix]{"version":"1.0"}
%---
