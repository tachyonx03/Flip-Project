function res = compareEstimator(runSim)
%COMPAREESTIMATOR  참값 자세 vs 팀원 추정기 자세 비교 그래프
%
%   compareEstimator()      워크스페이스에 있는 결과로 그림 (estimR 없으면 자동으로 시뮬 실행)
%   compareEstimator(true)  시뮬 먼저 돌리고 그림
%
%   참값   : log_states.DCM_be  (Airframe이 물리계산으로 아는 진짜 자세)
%   추정값 : estimR             (stateEstimator가 센서만 보고 추측한 자세)
%
%   둘 다 3x3 회전행렬이라 짐벌락 없음. 비교 지표 2개:
%     tilt = acos(R(3,3))  기울기. R(3,3)은 전치해도 같은 값이라 좌표규약과 무관
%     yaw  = atan2(...)    방위각. 규약(전치 여부)을 모르므로 두 가지 다 계산해
%                          참값과 잘 맞는 쪽을 자동 선택
%
%   출력 res : 요약 수치 구조체

if nargin < 1 || isempty(runSim), runSim = []; end
mdl = 'asbQuadcopter';

haveVars = evalin('base','exist(''log_states'',''var'')') && ...
           evalin('base','exist(''estimR'',''var'')');
if isempty(runSim), runSim = ~haveVars; end

if runSim
    load_system(mdl);
    oldPace = get_param(mdl,'EnablePacing');
    oldStop = get_param(mdl,'StopFcn');
    restore = onCleanup(@() set_param(mdl,'EnablePacing',oldPace,'StopFcn',oldStop));
    set_param(mdl,'EnablePacing','off');   % 실시간 묶기 해제 (안 하면 매우 느림)
    set_param(mdl,'StopFcn','');           % flipGraph가 stale 변수 읽는 것 방지
    fprintf('시뮬 실행 중 (StopTime=%s) ...\n', get_param(mdl,'StopTime'));
    out = sim(Simulink.SimulationInput(mdl));
    ls  = out.log_states;
    est = out.estimR;
    sen = out.sensor;
    assignin('base','log_states',ls);      % 이후 수동 분석용
    assignin('base','estimR',est);
    assignin('base','sensor',sen);
else
    ls  = evalin('base','log_states');
    est = evalin('base','estimR');
    sen = evalin('base','sensor');
end

% ---------- 자세 꺼내기 ----------
tT = ls.DCM_be.Time;   RT = double(ls.DCM_be.Data);   % 참값 3x3xN
tE = est.Time;         RE = double(est.Data);         % 추정 3x3xN

% ---------- 기울기 (좌표규약 무관) ----------
tiltT = squeeze(acos(min(1,max(-1, RT(3,3,:))))) * 180/pi;
tiltE = squeeze(acos(min(1,max(-1, RE(3,3,:))))) * 180/pi;

% ---------- 방위각 : 두 규약 다 계산 ----------
yawT  = unwrap(squeeze(atan2(RT(1,2,:), RT(1,1,:)))) * 180/pi;
yawEa = unwrap(squeeze(atan2(RE(1,2,:), RE(1,1,:)))) * 180/pi;  % 참값과 같은 규약
yawEb = unwrap(squeeze(atan2(RE(2,1,:), RE(1,1,:)))) * 180/pi;  % 전치 규약

% 참값 시간축으로 맞춰서 어느 규약이 맞는지 판정
ia = interp1(tE, yawEa, tT, 'linear', 'extrap');
ib = interp1(tE, yawEb, tT, 'linear', 'extrap');
ra = sqrt(mean((ia - yawT).^2));
rb = sqrt(mean((ib - yawT).^2));
if ra <= rb
    yawE = yawEa;  yawEi = ia;  conv = '동일 규약 atan2(R(1,2),R(1,1))';  yawRMS = ra;
else
    yawE = yawEb;  yawEi = ib;  conv = '전치 규약 atan2(R(2,1),R(1,1))';  yawRMS = rb;
end

tiltEi   = interp1(tE, tiltE, tT, 'linear', 'extrap');
tiltErr  = tiltEi - tiltT;
yawErr   = yawEi  - yawT;
tiltRMS  = sqrt(mean(tiltErr.^2));

% ---------- 플립 구간 (기울기 90도 초과) ----------
inFlip = tiltT > 90;
if any(inFlip)
    flipT = [tT(find(inFlip,1,'first')), tT(find(inFlip,1,'last'))];
else
    flipT = [];
end

% ---------- 그래프 ----------
% ---------- 자이로 포화 (추정 실패의 근본 원인) ----------
if isstruct(sen), tS = sen.time; sV = sen.signals.values; else, tS = sen.Time; sV = sen.Data; end
gyroSensor = sV(:,4:6);                       % 센서가 실제로 측정한 p,q,r
Wt = ls.Omega_body.Data;                      % 참 각속도
if ndims(Wt) == 3, Wt = squeeze(Wt)'; end
tWt = ls.Omega_body.Time;
gLim = 10;                                    % Sensors.IMU.gyroLimits (rad/s)
try, gl = evalin('base','Sensors.IMU.gyroLimits'); gLim = max(gl); catch, end
satFrac = mean(any(abs(gyroSensor) >= gLim*0.999, 2)) * 100;

f = findobj('Type','figure','Name','estimator_compare');
if isempty(f), f = figure('Name','estimator_compare','Color','k','Position',[60 40 1150 950]);
else, figure(f); clf(f); set(f,'Color','k'); end

sp(1) = subplot(4,1,1);
plot(tT, tiltT, 'w-', 'LineWidth', 1.6); hold on;
plot(tE, tiltE, 'y--', 'LineWidth', 1.4);
yline(90, ':', 'Color', [.6 .6 .6]);
ylabel('기울기 [deg]'); title('기울기(tilt) : 참값 vs 추정');
legend({'참값 DCM\_be','추정 estimR','90^\circ'}, 'Location','best');

sp(2) = subplot(4,1,2);
plot(tT, yawT, 'w-', 'LineWidth', 1.6); hold on;
plot(tE, yawE, 'y--', 'LineWidth', 1.4);
ylabel('방위각 [deg]'); title(sprintf('방위각(yaw) : 참값 vs 추정   [%s]', conv));
legend({'참값','추정'}, 'Location','best');

sp(3) = subplot(4,1,3);
plot(tT, tiltErr, 'w-', 'LineWidth', 1.4); hold on;
plot(tT, yawErr,  'y--','LineWidth', 1.4);
yline(0, ':', 'Color', [.6 .6 .6]);
ylabel('오차 [deg]'); title('추정 오차 (추정 - 참값)');
legend({'기울기 오차','방위각 오차'}, 'Location','best');

sp(4) = subplot(4,1,4);
plot(tWt, Wt(:,2), 'w-', 'LineWidth', 1.6); hold on;
plot(tS, gyroSensor(:,2), 'y--', 'LineWidth', 1.4);
yline( gLim, 'r:', 'LineWidth', 1.5);
yline(-gLim, 'r:', 'LineWidth', 1.5);
xlabel('시간 [s]'); ylabel('피치 각속도 q [rad/s]');
title(sprintf('근본 원인 : 자이로 포화 (한계 \\pm%g rad/s = \\pm%.0f deg/s)', gLim, gLim*180/pi));
legend({'참 각속도','자이로가 측정한 값','포화 한계'}, 'Location','best');

% 플립 구간 음영
for k = 1:numel(sp)
    if ~isempty(flipT)
        yl = ylim(sp(k));
        patch(sp(k), [flipT(1) flipT(2) flipT(2) flipT(1)], [yl(1) yl(1) yl(2) yl(2)], ...
              [1 .3 .3], 'FaceAlpha', .12, 'EdgeColor','none', 'HandleVisibility','off');
        uistack(findobj(sp(k),'Type','patch'),'bottom');
        ylim(sp(k), yl);
    end
    grid(sp(k),'on');
end

% 검은 배경 스타일
ax = findobj(f,'Type','axes');
set(ax,'Color','k','XColor','w','YColor','w','ZColor','w', ...
       'GridColor',[.8 .8 .8],'GridAlpha',.3);
set(findall(f,'Type','text'),'Color','w');
set(findobj(f,'Type','legend'),'TextColor','w','Color','k','EdgeColor','w');

% ---------- 요약 ----------
res = struct();
res.convention   = conv;
res.tiltRMS_deg  = tiltRMS;
res.yawRMS_deg   = yawRMS;
res.tiltMaxErr   = max(abs(tiltErr));
res.yawMaxErr    = max(abs(yawErr));
res.maxTiltTrue  = max(tiltT);
res.flipWindow   = flipT;
res.yawFinalTrue = yawT(end);
res.yawFinalEst  = yawEi(end);
res.gyroLimit    = gLim;
res.gyroTrueMax  = max(abs(Wt(:)));
res.gyroSatPct   = satFrac;

fprintf('\n===== 추정기 vs 참값 =====\n');
fprintf('  규약 판정        : %s\n', conv);
fprintf('  최대 기울기(참)  : %.1f deg\n', res.maxTiltTrue);
if ~isempty(flipT)
    fprintf('  플립 구간        : %.2f ~ %.2f s\n', flipT(1), flipT(2));
else
    fprintf('  플립 구간        : 없음 (90도 안 넘음)\n');
end
fprintf('  기울기 오차      : RMS %.2f deg / 최대 %.2f deg\n', res.tiltRMS_deg, res.tiltMaxErr);
fprintf('  방위각 오차      : RMS %.2f deg / 최대 %.2f deg\n', res.yawRMS_deg,  res.yawMaxErr);
fprintf('  최종 방위각      : 참 %.2f deg / 추정 %.2f deg\n', res.yawFinalTrue, res.yawFinalEst);
fprintf('  ---- 근본 원인 ----\n');
fprintf('  자이로 한계      : +-%g rad/s (+-%.0f deg/s)\n', gLim, gLim*180/pi);
fprintf('  실제 최대 각속도 : %.1f rad/s  --> 한계의 %.1f 배\n', res.gyroTrueMax, res.gyroTrueMax/gLim);
fprintf('  포화 시간 비율   : %.2f %% (전체 시간 중)\n', satFrac);
fprintf('  판정 기준        : 옛 오일러 추정기는 여기서 수만~수십만 deg로 발산했음\n');
end
