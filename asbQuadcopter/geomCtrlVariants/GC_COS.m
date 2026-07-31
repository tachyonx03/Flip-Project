% GC_COS
% 연속: w_yaw = max(0,min(1,R33)), 래치 미사용
% 2026-07-29 tilt-torsion 비교 실험본.
% 적용법: sfroot 로 EMChart 잡아 .Script 에 이 파일 내용 대입
%   ch = sfroot.find('-isa','Stateflow.EMChart','Path', ...
%        'flightController/Flight Controller/Attitude/MATLAB Function');
%   ch.Script = fileread('GC_COS.m');

function [tau_pitch, tau_roll, tau_yaw] = GeometricController(R, W, refAtt)

    % ===== 고정 파라미터 =====
    kR = 0.06;   kW = 0.006;
    J  = diag([5.82857e-5, 7.16914e-5, 1e-4]);
    a_max    = 394;      % rad/s^2   = tau_max/Jyy, 포화시 각감속도
    tau_max  = 0.0282;   % N*m       = 2L(fmax-fmin)
    R33_exit = 0.707;    % cos(45deg), 해제 기울기 (추력예산 33% 확보)

    R_b2e = double(R.');
    Wd    = double(W);
    R33   = R_b2e(3,3);

    % ===== 물리량 =====
    w_tilt = hypot(Wd(1), Wd(2));                  % 기울임 각속도 (yaw 제외)
    theta  = acos(min(1, max(-1, R33)));           % 기울기 0..pi
    w_crit = sqrt(2*a_max*max(0, pi - theta));     % 플립 완주 판정선
    w_sat  = tau_max / kW;                         % D항 포화 경계 (~4.7)

    % ===== 플립 모드 (래치 + 히스테리시스) =====
    persistent inFlip
    if isempty(inFlip)
        inFlip = false;
    end

    if ~inFlip
        % 진입: 90° 넘었거나, 최대제동으로도 못 멈추는 각속도
        inFlip = (R33 < 0) || (w_tilt > w_crit);
    else
        % 해제: 45° 안으로 복귀 AND 제어기가 선형영역 복귀
        inFlip = ~( (R33 > R33_exit) && (w_tilt < w_sat) );
    end

    % ===== R_des =====
    if inFlip
        R_des = eye(3);                            % 플립 중: 수평 고정
    else
        p = double(refAtt(1));
        r = double(refAtt(2));
        Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
        Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
        R_des = Ry*Rx;
    end

    % ===== 기하 제어 =====
    W_des = [0; 0; 0];
    eR  = vee(0.5*(R_des.'*R_b2e - R_b2e.'*R_des));
    eW  = Wd - R_b2e.'*R_des*W_des;
    % ===== tilt-torsion 분리 =====
    % w_yaw = cos(기울기) = 추력 여유 척도. 90도 넘으면 0.
    w_yaw = max(0, min(1, R33));
    eR(3) = w_yaw * eR(3);

    tau = -kR*eR - kW*eW + cross(Wd, J*Wd);

    tau_pitch = single(tau(2));
    tau_roll  = single(tau(1));
    tau_yaw   = single(tau(3));
end

function v = vee(M)
    v = [M(3,2); M(1,3); M(2,1)];
end