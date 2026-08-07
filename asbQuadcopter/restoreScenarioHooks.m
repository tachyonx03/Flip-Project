function status = restoreScenarioHooks(action)
%RESTORESCENARIOHOOKS  flipScenarioUI 가 쓰는 airframe 훅을 복구하거나 걷어낸다.
%
%   flipScenarioUI 의 두 시나리오는 nonlinearAirframe 안의 훅 3종으로 동작한다.
%   이 훅들은 커밋 dd12f10 ("Remove impact disturbance from airframe", 2026-07-31)
%   에서 삭제됐고, 그 뒤로 UI 를 돌리면 외란이 하나도 없는 맹탕 호버만 나왔다.
%   (UI 가 Step/Step1 까지 [0 0 0] 으로 죽이므로 토크 펄스마저 안 나간다.)
%
%     ① ImpactGen + ImpactClock                     → 시나리오 ① 타격
%        impact_t0 / impact_dur / impact_F / impact_r 를 읽어 몸체 힘 F 와
%        그에 딸린 토크 r x F 를 만든다. F 는 AC model 의 force_ext(7번 입력)로,
%        토크는 Sum1 을 거쳐 tau_ext(6번 입력)로 들어간다.
%
%     ② MotorGate + MotorGateCmp / MotorGateDelay / MotorGateZero
%        t < release_delay 동안 모터 명령을 Vehicle.Motor.minLimit(아이들)로 묶는다.
%        0 을 명령하면 [10,500] 범위 밖이라 오히려 큰 추력이 나오므로 minLimit 을 쓴다.
%
%     ③ SO3_dynamics 의 초기조건 파라미터            → 시나리오 ② 초기 자세
%        하드코딩된 eye(3) / zeros(3,1) / [57;95;-0.046] 대신
%        init_Cbe / init_W / init_xN / init_vN 을 읽는다. 이 네 개는 입력 포트가
%        아니라 Parameter 스코프라서 블록 배선은 전혀 바뀌지 않는다.
%
%     ④ ImpactClock 은 반드시 Digital Clock(Ts). ①②가 공유한다.
%        연속시간 Clock 을 쓰면 airframe 전체가 연속시간으로 끌려가 플랜트가
%        3배 빨리 적분된다 (i_clockDiscrete 주석 참고).
%
%   안전성: startVars.m 의 기본값(impact_F=0, release_delay=0,
%   init_* = 기존 하드코딩값과 동일)이 전부 무동작이라, 복구 전후로
%   flipSweep / ttRun / flipYawDiag 의 결과는 달라지지 않는다.
%
%   ※ dd12f10 당시 Sum1 의 부호가 '+ + +' 였다. Step - Step1 로 펄스를 만드는
%     구조인데 부호가 + 라 임펄스가 아니라 계단(0 -> A -> 2A 영구)이 나갔다.
%     여기서는 그 버그를 되살리지 않고 '+ - +' (Step, -Step1, +impact_tau)로 넣는다.
%
%   사용법
%     restoreScenarioHooks             현재 상태만 점검해서 출력 (기본값)
%     restoreScenarioHooks('session')  훅 복구, 단 파일은 저장하지 않음
%     restoreScenarioHooks('apply')    훅 복구 + nonlinearAirframe 저장
%     restoreScenarioHooks('undo')     훅 제거(dd12f10 상태로) + 저장
%     s = restoreScenarioHooks(...)    결과를 구조체로 반환 (출력 없음)
%
%   'session' 은 'apply' 와 똑같이 훅을 넣지만 save_system 을 하지 않는다.
%   nonlinearAirframe.slx 는 팀원(dd12f10)이 훅을 걷어낸 상태 그대로 디스크에
%   남고, 열려 있는 세션에서만 시나리오가 돈다. 공용 바이너리 모델에 diff 를
%   남기지 않으므로 병합 충돌 없이 혼자 시나리오를 돌려볼 때 이쪽을 쓴다.
%   단 MATLAB 을 닫거나 bdclose 하면 사라지므로 그때마다 다시 불러야 한다.
%
%   See also FLIPSCENARIOUI, STARTVARS.

if nargin < 1 || isempty(action), action = 'check'; end
action = validatestring(action, {'check','apply','session','undo'});

MDL = 'nonlinearAirframe';
NL  = [MDL '/Nonlinear'];
verbose = (nargout == 0);

load_system(MDL);

% ---- 블록 좌표 (e65f985 원본과 동일하게 되돌린다) ----
POS.ImpactClock    = [215, 615, 235, 635];
POS.ImpactGen      = [290, 599, 400, 666];
POS.MotorGateDelay = [215, 690, 265, 710];
POS.MotorGateCmp   = [300, 672, 330, 703];
POS.MotorGateZero  = [215, 735, 265, 755];
POS.MotorGate      = [380, 660, 420, 760];
POS.Constant5      = [ 80, 210, 110, 240];

status = i_survey(NL);

switch action
    case 'check'
        if verbose, i_report(status); end
        return
    case {'apply','session'}
        changed = i_apply(NL, POS, status);
    case 'undo'
        changed = i_undo(NL, POS, status);
end

keepOnDisk = strcmp(action,'session');   % 세션 한정: 저장하지 않는다

if changed
    if keepOnDisk
        note = sprintf('%s 는 저장하지 않았습니다 (이 세션에서만 유효).', MDL);
    else
        save_system(MDL);
        note = sprintf('%s 저장했습니다.', MDL);
    end
    status = i_survey(NL);
    if verbose
        fprintf('[restoreScenarioHooks] %s 완료 — %s\n', action, note);
        i_report(status);
    end
else
    if verbose
        fprintf('[restoreScenarioHooks] 바꿀 것이 없습니다 (이미 %s 상태).\n', action);
        i_report(status);
    end
end
end

% ======================================================================
%  현재 상태 조사
% ======================================================================
function s = i_survey(NL)
s.impact    = i_has([NL '/ImpactGen']) && i_has([NL '/ImpactClock']);
s.motorGate = i_has([NL '/MotorGate']) && i_has([NL '/MotorGateCmp']) && ...
              i_has([NL '/MotorGateDelay']) && i_has([NL '/MotorGateZero']);
s.initState = contains(i_script([NL '/MATLAB Function']), 'init_Cbe');
s.clock     = i_clockDiscrete([NL '/ImpactClock']);
s.allReady  = s.impact && s.motorGate && s.initState && s.clock;
end

% ImpactClock 은 반드시 Digital Clock(Ts) 여야 한다. 연속시간 Clock 을 쓰면
% Step/Step1 이 상속(-1) 샘플타임이라 그 하나가 AC model·Actuators·
% SO3_dynamics 까지 airframe 전체를 연속시간으로 끌고 간다. SO3_dynamics 는
% dt=0.005 를 하드코딩한 채 persistent 로 상태를 들고 있어서, 고정스텝 ode3
% 의 스테이지마다(스텝당 3회) 불리며 매번 5 ms 를 적분한다. 플랜트가 3배
% 빨리 도는 셈이라 impact_F=0 인 무동작 기본값에서도 기체가 1.5초 만에
% 뒤집혀 1300 m 를 날아간다. (2026-08-07 실측: Digital Clock 으로 바꾸면
% 훅 없는 기준 비행과 소수점까지 동일한 결과가 나온다.)
function tf = i_clockDiscrete(path)
tf = false;
if getSimulinkBlockHandle(path) <= 0, return; end
tf = strcmp(get_param(path,'BlockType'), 'DigitalClock');
end

function i_report(s)
fprintf('  ① 타격 (ImpactGen/ImpactClock)        : %s\n', i_yn(s.impact));
fprintf('  ② 모터 정지 (MotorGate 4종)           : %s\n', i_yn(s.motorGate));
fprintf('  ③ 초기 자세 (SO3_dynamics init_* 인자): %s\n', i_yn(s.initState));
fprintf('  ④ ImpactClock 이 이산(Digital, Ts)    : %s\n', i_yn(s.clock));
if s.allReady
    fprintf('  => flipScenarioUI 를 그대로 실행하면 됩니다.\n');
else
    fprintf('  => restoreScenarioHooks(''apply'') 를 실행하세요.\n');
end
end

function s = i_yn(tf)
if tf, s = '있음'; else, s = '없음'; end
end

function tf = i_has(path)
tf = getSimulinkBlockHandle(path) > 0;
end

function i_addClock(NL, POS)
add_block('simulink/Sources/Digital Clock', [NL '/ImpactClock'], ...
    'Position', POS.ImpactClock, 'SampleTime', 'Ts');
end

function s = i_script(path)
s = '';
ch = sfroot().find('-isa','Stateflow.EMChart','Path',path);
if ~isempty(ch), s = ch(1).Script; end
end

% ======================================================================
%  복구
% ======================================================================
function changed = i_apply(NL, POS, st)
changed = false;

% ---------- ④ 레거시 연속시간 Clock 교체 ----------
% 이 파일의 예전 버전은 ImpactClock 을 연속시간 Clock 으로 넣었다. 그 상태로
% 저장된 모델을 만나면 여기서 Digital Clock(Ts) 로 갈아끼운다. 이유는
% i_clockDiscrete 주석 참고.
if i_has([NL '/ImpactClock']) && ~st.clock
    i_dropLine(NL, 'ImpactClock/1', 'ImpactGen/1');
    i_dropLine(NL, 'ImpactClock/1', 'MotorGateCmp/1');
    i_dropBlock([NL '/ImpactClock']);
    i_addClock(NL, POS);
    if i_has([NL '/ImpactGen'])
        add_line(NL, 'ImpactClock/1', 'ImpactGen/1', 'autorouting','on');
    end
    if i_has([NL '/MotorGateCmp'])
        add_line(NL, 'ImpactClock/1', 'MotorGateCmp/1', 'autorouting','on');
    end
    changed = true;
end

% ---------- ③ SO3_dynamics 초기조건 파라미터화 ----------
if ~st.initState
    i_patchPlant(NL, true);
    changed = true;
end

% ---------- ① ImpactGen ----------
if ~st.impact
    if ~i_has([NL '/ImpactClock'])
        i_addClock(NL, POS);
    end

    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [NL '/ImpactGen'], 'Position', POS.ImpactGen);
    ch = sfroot().find('-isa','Stateflow.EMChart','Path',[NL '/ImpactGen']);
    ch.Script = i_impactScript();
    i_setParamScope(ch, {'impact_t0','impact_dur','impact_F','impact_r'});

    % force_ext : Constant5([0 0 0]) 를 걷어내고 ImpactGen 의 F 로 교체
    if i_has([NL '/Constant5'])
        i_dropLine(NL, 'Constant5/1', 'AC model/7');
        delete_block([NL '/Constant5']);
    end
    add_line(NL, 'ImpactClock/1', 'ImpactGen/1', 'autorouting','on');
    add_line(NL, 'ImpactGen/1',   'AC model/7',  'autorouting','on');

    % tau_ext : Sum1 을 Step - Step1 + impact_tau 로 (부호 주의)
    set_param([NL '/Sum1'], 'Inputs', '|+-+');
    add_line(NL, 'ImpactGen/2', 'Sum1/3', 'autorouting','on');
    changed = true;
end

% ---------- ② MotorGate ----------
if ~st.motorGate
    add_block('simulink/Sources/Constant', [NL '/MotorGateDelay'], ...
        'Position', POS.MotorGateDelay, 'Value', 'release_delay');
    add_block('simulink/Sources/Constant', [NL '/MotorGateZero'], ...
        'Position', POS.MotorGateZero, ...
        'Value', 'Vehicle.Motor.minLimit*ones(1,4)', 'OutDataTypeStr', 'single');
    add_block('simulink/Logic and Bit Operations/Relational Operator', ...
        [NL '/MotorGateCmp'], 'Position', POS.MotorGateCmp, ...
        'Operator', '>=', 'InputSameDT', 'off', 'OutDataTypeStr', 'boolean');
    add_block('simulink/Signal Routing/Switch', [NL '/MotorGate'], ...
        'Position', POS.MotorGate, 'Criteria', 'u2 >= Threshold', ...
        'Threshold', '0.5', 'InputSameDT', 'off', 'SaturateOnIntegerOverflow', 'off');

    % ImpactClock 이 방금 생겼을 수도, 이미 있었을 수도 있다
    if ~i_has([NL '/ImpactClock'])
        i_addClock(NL, POS);
    end

    i_dropLine(NL, 'Actuators/1', 'AC model/1');
    add_line(NL, 'ImpactClock/1',    'MotorGateCmp/1', 'autorouting','on');
    add_line(NL, 'MotorGateDelay/1', 'MotorGateCmp/2', 'autorouting','on');
    add_line(NL, 'Actuators/1',      'MotorGate/1',    'autorouting','on');
    add_line(NL, 'MotorGateCmp/1',   'MotorGate/2',    'autorouting','on');
    add_line(NL, 'MotorGateZero/1',  'MotorGate/3',    'autorouting','on');
    add_line(NL, 'MotorGate/1',      'AC model/1',     'autorouting','on');
    changed = true;
end
end

% ======================================================================
%  제거 (dd12f10 상태로)
% ======================================================================
function changed = i_undo(NL, POS, st)
changed = false;

if st.motorGate
    i_dropLine(NL, 'MotorGate/1', 'AC model/1');
    for b = {'MotorGate','MotorGateCmp','MotorGateDelay','MotorGateZero'}
        i_dropBlock([NL '/' b{1}]);
    end
    add_line(NL, 'Actuators/1', 'AC model/1', 'autorouting','on');
    changed = true;
end

if st.impact
    i_dropLine(NL, 'ImpactGen/1', 'AC model/7');
    i_dropLine(NL, 'ImpactGen/2', 'Sum1/3');
    set_param([NL '/Sum1'], 'Inputs', '|+-');
    i_dropBlock([NL '/ImpactGen']);
    if ~i_has([NL '/Constant5'])
        add_block('simulink/Sources/Constant', [NL '/Constant5'], ...
            'Position', POS.Constant5, 'Value', '[0 0 0]');
    end
    add_line(NL, 'Constant5/1', 'AC model/7', 'autorouting','on');
    changed = true;
end

% ImpactClock 은 둘 다 쓰므로 마지막에 정리
i_dropBlock([NL '/ImpactClock']);

if st.initState
    i_patchPlant(NL, false);
    changed = true;
end
end

% ======================================================================
%  SO3_dynamics 스크립트 패치
%  주석(한글 포함)을 건드리지 않도록 딱 5군데만 문자열 치환한다.
% ======================================================================
function i_patchPlant(NL, toParameterized)
ch = sfroot().find('-isa','Stateflow.EMChart','Path',[NL '/MATLAB Function']);
if isempty(ch)
    error('restoreScenarioHooks:noPlant', '%s/MATLAB Function 을 찾지 못했습니다.', NL);
end
ch = ch(1);

pairs = {
  'function [Ve, Xe, DCM_out, Vb, W_out, accel, k_diff] = SO3_dynamics(F_cg, M_cg)', ...
  'function [Ve, Xe, DCM_out, Vb, W_out, accel, k_diff] = SO3_dynamics(F_cg, M_cg, init_Cbe, init_W, init_xN, init_vN)'
  'Cbe = eye(3);',              'Cbe = init_Cbe;'
  'W   = zeros(3,1);',          'W   = init_W;'
  'xN  = [57; 95; -0.046];',    'xN  = init_xN;'
  'vN  = zeros(3,1);',          'vN  = init_vN;'
};
if toParameterized, from = 1; to = 2; else, from = 2; to = 1; end

src = ch.Script;
for k = 1:size(pairs,1)
    n = numel(strfind(src, pairs{k,from}));
    if n ~= 1
        error('restoreScenarioHooks:ambiguousPatch', ...
            ['SO3_dynamics 에서 "%s" 가 %d 번 나옵니다 (1번이어야 함).\n' ...
             '플랜트 코드가 그 사이에 바뀐 것 같습니다. 수동으로 확인하세요.'], ...
            pairs{k,from}, n);
    end
    src = strrep(src, pairs{k,from}, pairs{k,to});
end
ch.Script = src;

if toParameterized
    i_setParamScope(ch, {'init_Cbe','init_W','init_xN','init_vN'});
end
end

% ======================================================================
%  MATLAB Function 인자를 입력 포트가 아닌 Parameter 로 바꾼다
%  (이래야 워크스페이스 변수를 그대로 읽고, 배선이 안 늘어난다)
% ======================================================================
function i_setParamScope(ch, names)
d = ch.find('-isa','Stateflow.Data');
for k = 1:numel(d)
    if any(strcmp(d(k).Name, names))
        d(k).Scope = 'Parameter';
    end
end
end

% ======================================================================
%  없는 선/블록을 지우려 해도 안 터지게
% ======================================================================
function i_dropLine(sys, src, dst)
try
    delete_line(sys, src, dst);
catch
end
end

function i_dropBlock(path)
if getSimulinkBlockHandle(path) > 0
    try
        delete_block(path);
    catch
    end
end
end

% ======================================================================
%  ImpactGen 본문 (f57f5b5 원본과 동일한 3x1 열벡터 출력.
%  AC model 의 tau_ext/force_ext 인포트가 둘 다 PortDimensions [3 1] 이다.)
% ======================================================================
function s = i_impactScript()
s = sprintf('%s\n', ...
'function [F, tau] = fcn(t, impact_t0, impact_dur, impact_F, impact_r)', ...
'%#codegen', ...
'% Short contact starting at impact_t0 and lasting impact_dur.', ...
'% impact_F is the contact force in the BODY frame and impact_r is the hit', ...
'% point relative to the CG, also BODY, so the torque it applies is r x F.', ...
'% Zero impact_F (the default) leaves the airframe untouched.', ...
'F   = zeros(3,1);', ...
'tau = zeros(3,1);', ...
'if (t >= impact_t0) && (t < impact_t0 + impact_dur)', ...
'    fb  = impact_F(:);', ...
'    rb  = impact_r(:);', ...
'    F   = fb;', ...
'    tau = cross(rb, fb);', ...
'end');
end
