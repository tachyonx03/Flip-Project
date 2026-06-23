% addForceDisturbance.m
% nonlinearAirframe의 AC model에 외부 힘 교란 입력(force_ext)을 추가합니다.
%
% 수정 내용:
%   Applied Force Calculation (out1) --> [Sum] --> F_cg
%                              force_ext (Inport) --/
%
% Copyright 2024

modelName = 'nonlinearAirframe';
acPath    = [modelName '/Nonlinear/AC model'];

%% 1. 모델 로드
load_system(modelName);

%% 2. 기존 연결 (Applied Force Calculation → F_cg) 삭제
delete_line(acPath, 'Applied Force Calculation/1', 'F_cg/1');

%% 3. Sum 블록 추가
sumPos = [800, 275, 820, 305];
add_block('simulink/Math Operations/Sum', [acPath '/Force Sum'], ...
    'Position',  sumPos,        ...
    'Inputs',    '++',          ...
    'IconShape', 'rectangular');

%% 4. Inport(force_ext) 추가 — [3x1] N, body frame
inportPos = [610, 335, 640, 349];
add_block('simulink/Sources/In1', [acPath '/force_ext'], ...
    'Position',       inportPos,  ...
    'PortDimensions', '[3 1]',    ...
    'OutDataTypeStr', 'double');

% 포트 번호를 tau_ext(6번) 다음으로 설정
set_param([acPath '/force_ext'], 'Port', '7');

%% 5. 새 연결
add_line(acPath, 'Applied Force Calculation/1', 'Force Sum/1', 'autorouting', 'on');
add_line(acPath, 'force_ext/1',                 'Force Sum/2', 'autorouting', 'on');
add_line(acPath, 'Force Sum/1',                 'F_cg/1',      'autorouting', 'on');

%% 6. 상위 블록(Nonlinear)에 force_ext 포트 연통 — Constant 0으로 막아둠
nonlinPath = [modelName '/Nonlinear'];
constPos   = [140, 273, 200, 293];
add_block('simulink/Sources/Constant', [nonlinPath '/force_ext_zero'], ...
    'Position',       constPos,     ...
    'Value',          '[0;0;0]',    ...
    'OutDataTypeStr', 'double');

add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');

%% 7. 저장
save_system(modelName);
disp('완료: nonlinearAirframe에 force_ext 입력이 추가됐습니다.');
disp('교란 주입: setForceDisturbance(type, axis, magnitude, ...) 사용');
