function [T, tau] = motorSat(T, tau, p)
% motorSat  모터 추력 한계를 반영한 액추에이터 모델
% 평면 모델이라 실제 4로터를 [앞 2개 / 뒤 2개] 덩어리로 묶어 취급
%       T   : 총 추력 [N]
%       tau : 피치 토크 [N·m]
%       p.l    : 피치 모멘트암 [m]
%       p.fmax : 모터당 최대추력 [N]
%       p.fmin : 모터당 최소추력 [N] (idle)
%
%   핵심: T와 tau는 커플링돼 있다. 큰 tau를 짜내려면 한쪽 덩어리를 최대,
%   반대쪽을 최소로 몰아야 하므로 총 추력 T가 부족해진다
%   → 플립 도는 동안 고도가 떨어지는 현실적 현상이 자동으로 나타난다.

    Fmax = 2*p.fmax;          % 한 덩어리(2로터) 최대추력
    Fmin = 2*p.fmin;          % 한 덩어리(2로터) 최소추력

    % ① 명령 (T, tau)를 앞/뒤 추력으로 분해
    Ff = (T + tau/p.l)/2;     % 앞 덩어리 추력
    Fb = (T - tau/p.l)/2;     % 뒤 덩어리 추력

    % ② 각 덩어리를 물리적 한계로 clamp
    Ff = min(max(Ff, Fmin), Fmax);
    Fb = min(max(Fb, Fmin), Fmax);

    % ③ 실제 낼 수 있는 값으로 재합성
    T   = Ff + Fb;
    tau = p.l*(Ff - Fb);
end

% m    = 0.063 kg         질량
% J    = 7.169e-5 kg·m²   피치 관성 (Jyy)
% g    = 9.81 m/s²
% L    = 0.0441 m         피치 모멘트암
% fmax = 0.3266 N         모터당 최대추력
% fmin = 0.0065 N         모터당 최소추력 (idle)
