% setPayloadDisturbance.m
% 페이로드 외란: 질량과 관성 파라미터를 변경하여 페이로드 탑재/투하를 모사합니다.
%
% 사용법:
%   setPayloadDisturbance('add',    0.02)    % 20g 페이로드 추가
%   setPayloadDisturbance('remove', 0.02)    % 20g 페이로드 제거
%   setPayloadDisturbance('reset')           % 기본값(Mambo 원래 스펙)으로 복원
%
% 원리: Vehicle.Airframe.mass, Vehicle.Airframe.inertia를 변경 후
%       시뮬레이션을 다시 실행하면 파라미터 변화 효과를 확인할 수 있음.
%
% 기본 Mambo 스펙:
%   mass    = 0.063 kg
%   inertia = diag([5.83e-5, 7.17e-5, 1.0e-4]) kg*m^2

function setPayloadDisturbance(mode, payload_mass)

% 기본 Mambo 스펙
base_mass    = 0.063;
base_inertia = diag([0.0000582857, 0.0000716914, 0.0001]);

if strcmp(mode, 'reset')
    Vehicle.Airframe.mass    = base_mass;
    Vehicle.Airframe.inertia = base_inertia;
    assignin('base', 'Vehicle', evalin('base', 'Vehicle'));
    % 직접 워크스페이스 변수 갱신
    ws_vehicle = evalin('base', 'Vehicle');
    ws_vehicle.Airframe.mass    = base_mass;
    ws_vehicle.Airframe.inertia = base_inertia;
    assignin('base', 'Vehicle', ws_vehicle);
    fprintf('페이로드 초기화: mass=%.3fkg (기본값)\n', base_mass);
    return;
end

if nargin < 2
    error('add/remove 모드는 payload_mass(kg) 필요');
end

ws_vehicle = evalin('base', 'Vehicle');

switch lower(mode)
    case 'add'
        new_mass = ws_vehicle.Airframe.mass + payload_mass;
        % 관성도 비례해서 증가 (페이로드가 중심에 붙는다고 가정)
        ratio = new_mass / ws_vehicle.Airframe.mass;
        new_inertia = ws_vehicle.Airframe.inertia * ratio;
        ws_vehicle.Airframe.mass    = new_mass;
        ws_vehicle.Airframe.inertia = new_inertia;
        assignin('base', 'Vehicle', ws_vehicle);
        fprintf('페이로드 추가: mass %.3f → %.3f kg (+%.1fg)\n', ...
            new_mass - payload_mass, new_mass, payload_mass*1000);

    case 'remove'
        new_mass = ws_vehicle.Airframe.mass - payload_mass;
        if new_mass <= 0
            error('제거 후 질량이 0 이하: 현재 %.3fkg에서 %.3fkg 제거 불가', ...
                ws_vehicle.Airframe.mass, payload_mass);
        end
        ratio = new_mass / ws_vehicle.Airframe.mass;
        new_inertia = ws_vehicle.Airframe.inertia * ratio;
        ws_vehicle.Airframe.mass    = new_mass;
        ws_vehicle.Airframe.inertia = new_inertia;
        assignin('base', 'Vehicle', ws_vehicle);
        fprintf('페이로드 제거: mass %.3f → %.3f kg (-%.1fg)\n', ...
            new_mass + payload_mass, new_mass, payload_mass*1000);

    otherwise
        error('mode는 add/remove/reset 중 하나');
end

fprintf('→ 시뮬레이션 다시 실행하면 변경된 파라미터로 동작합니다.\n');
end
