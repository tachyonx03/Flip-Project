function m = ttRun(A, stopT, tag, quiet)
%TTRUN  대각선/단축 외란 펄스 1회 시뮬 + 지표·트레이스 수집.
%
%   m = ttRun([Ar Ap])                 외란 [Ar Ap 0], t=8~8.02, StopTime 20
%   m = ttRun([Ar Ap], stopT, tag)     StopTime·라벨 지정
%   m = ttRun(..., true)               콘솔 출력 끔
%
%   외란 주입점 = nonlinearAirframe/Nonlinear 의 Step - Step1 (0.02초 사각펄스).
%   호출 후 Step/Step1/StopTime/Pacing/StopFcn 전부 onCleanup 으로 원복됨.
%
%   반환 구조체 m:
%     t tilt yaw dev Wn         시계열 (그래프용)
%     maxTilt nCross90 finalTilt
%     maxYaw finalYaw
%     maxW finalW
%     maxDev finalDev
%
%   지표 해석
%     nCross90  기울기가 90도를 아래->위로 넘은 횟수 = 텀블 횟수
%     maxDev    시작 위치에서 수평으로 가장 멀리 밀려난 거리 [m]
%     finalDev  StopTime 시점 잔여 위치오차 [m]. 0 이면 제자리 복귀 성공
%
%   예)
%     r = ttRun([0.22 0.22], 20, 'diag0.22');
%     for k=1:numel(Avec), S(k)=ttRun([Avec(k) Avec(k)],20,sprintf('a%.2f',Avec(k))); end
%
%   참고: 시뮬 속도를 위해 EnablePacing 을 강제로 off 함 (원복됨).
%   See also TTPLOT4, FLIPSWEEP, COMPAREESTIMATOR.

if nargin < 2 || isempty(stopT), stopT = 20; end
if nargin < 3, tag = ''; end
if nargin < 4, quiet = false; end

mdl = 'asbQuadcopter'; af = 'nonlinearAirframe';
load_system(mdl); load_system(af);
S  = 'nonlinearAirframe/Nonlinear/Step';
S1 = 'nonlinearAirframe/Nonlinear/Step1';

oA  = get_param(S,'After');      oA1 = get_param(S1,'After');
oST = get_param(mdl,'StopTime'); oPC = get_param(mdl,'EnablePacing');
oFC = get_param(mdl,'StopFcn');
c  = onCleanup(@() set_param(S,'After',oA));                               %#ok<NASGU>
c1 = onCleanup(@() set_param(S1,'After',oA1));                             %#ok<NASGU>
c2 = onCleanup(@() set_param(mdl,'StopTime',oST, ...
                             'EnablePacing',oPC,'StopFcn',oFC));           %#ok<NASGU>

str = sprintf('[%g %g 0]', A(1), A(2));
set_param(S,'After',str); set_param(S1,'After',str);
set_param(mdl,'StopTime',num2str(stopT),'EnablePacing','off','StopFcn','');

ws = warning('off','all');
out = sim(Simulink.SimulationInput(mdl));
warning(ws);
ls = out.log_states;

R = double(ls.DCM_be.Data);  t = ls.DCM_be.Time;
W = double(ls.Omega_body.Data); if ndims(W)==3, W = squeeze(W)'; end
X = double(ls.X_ned.Data);      if ndims(X)==3, X = squeeze(X)'; end

% 기울기: R(3,3) 은 전치 규약과 무관하므로 좌표계 헷갈릴 걱정 없음
tilt = squeeze(acosd(min(1,max(-1,R(3,3,:)))));
yaw  = unwrap(squeeze(atan2(R(1,2,:),R(1,1,:))))*180/pi;
dev  = hypot(X(:,1)-X(1,1), X(:,2)-X(1,2));

m = struct();
m.tag = tag;  m.A = A;  m.stopT = stopT;
m.t = t;  m.tilt = tilt;  m.yaw = yaw;  m.dev = dev;  m.Wn = vecnorm(W,2,2);
m.maxTilt   = max(tilt);
m.nCross90  = numel(find(diff(tilt>90)==1));
m.finalTilt = tilt(end);
m.maxYaw    = max(abs(yaw));
m.finalYaw  = yaw(end);
m.maxW      = max(m.Wn);
m.finalW    = m.Wn(end);
m.maxDev    = max(dev);
m.finalDev  = dev(end);

if ~quiet
    fprintf(['%-12s A=[%g %g]  maxTilt %6.1f  x90 %2d  finalTilt %6.2f  ' ...
             '|maxYaw| %7.1f  maxDev %8.2f  finalDev %8.2f  finalW %.4f\n'], ...
        tag, A(1), A(2), m.maxTilt, m.nCross90, m.finalTilt, ...
        m.maxYaw, m.maxDev, m.finalDev, m.finalW);
end
end
