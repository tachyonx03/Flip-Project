% setTorqueDisturbance.m
% 시뮬레이션 전에 교란 신호를 설정합니다.
%
% 사용법:
%   setTorqueDisturbance('constant', 'roll', 0.002)
%   setTorqueDisturbance('step',     'pitch', 0.001, 5, 10)
%   setTorqueDisturbance('sine',     'roll',  0.001, 2)   % 2Hz
%   setTorqueDisturbance('off')
%
% Copyright 2024

function setTorqueDisturbance(type, axis, magnitude, varargin)

modelName  = 'nonlinearAirframe';
blockPath  = [modelName '/Nonlinear/tau_ext_zero'];

if strcmp(type, 'off')
    nonlinPath = [modelName '/Nonlinear'];
    blockType = get_param(blockPath, 'BlockType');
    if ~strcmp(blockType, 'Constant')
        delete_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6');
        delete_block(blockPath);
        add_block('simulink/Sources/Constant', ...
            [nonlinPath '/tau_ext_zero'], ...
            'Position', [140, 243, 200, 263], ...
            'Value', '[0;0;0]');
        add_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6', 'autorouting','on');
        save_system(modelName);
    else
        set_param(blockPath, 'Value', '[0;0;0]');
    end
    fprintf('교란 OFF\n');
    return;
end

% 축 벡터 설정
switch lower(axis)
    case 'roll',  axVec = '[1;0;0]';  axName = 'Roll(x)';
    case 'pitch', axVec = '[0;1;0]';  axName = 'Pitch(y)';
    case 'yaw',   axVec = '[0;0;1]';  axName = 'Yaw(z)';
    otherwise,    error('axis는 roll/pitch/yaw 중 하나');
end

switch lower(type)
    case 'constant'
        % 상수 토크
        val = sprintf('[%f;%f;%f]', ...
            magnitude*(strcmp(axis,'roll')), ...
            magnitude*(strcmp(axis,'pitch')), ...
            magnitude*(strcmp(axis,'yaw')));
        set_param(blockPath, 'Value', val);
        fprintf('교란 설정: %s축 %.4f N*m 상수\n', axName, magnitude);

    case 'step'
        % Step 신호: t_start, t_end 지정
        % varargin{1} = t_start, varargin{2} = t_end
        if numel(varargin) < 2
            error('step 타입은 t_start, t_end 필요');
        end
        t_start = varargin{1};
        t_end   = varargin{2};
        % Simulink Constant를 timeseries로 교체하는 대신
        % MATLAB workspace 변수로 넘기는 방식 사용
        t  = [0, t_start-1e-6, t_start, t_end, t_end+1e-6, 1000];
        mag = [0, 0, magnitude, magnitude, 0, 0];

        switch lower(axis)
            case 'roll',  ts = timeseries([mag;zeros(1,6);zeros(1,6)]', t');
            case 'pitch', ts = timeseries([zeros(1,6);mag;zeros(1,6)]', t');
            case 'yaw',   ts = timeseries([zeros(1,6);zeros(1,6);mag]', t');
        end
        ts.Name = 'tau_ext_ts';
        assignin('base', 'tau_ext_ts', ts);

        % Constant 블록을 From Workspace 블록으로 교체
        nonlinPath = [modelName '/Nonlinear'];
        delete_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6');
        delete_block(blockPath);

        add_block('simulink/Sources/From Workspace', ...
            [nonlinPath '/tau_ext_zero'], ...
            'Position',         [140, 243, 200, 263], ...
            'VariableName',     'tau_ext_ts',         ...
            'OutDataTypeStr',   'double',              ...
            'Interpolate',      'on');
        add_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6', 'autorouting','on');
        save_system(modelName);
        fprintf('교란 설정: %s축 %.4f N*m, t=%.1f~%.1fs\n', axName, magnitude, t_start, t_end);

    case 'sine'
        % 정현파: varargin{1} = 주파수 (Hz)
        if numel(varargin) < 1
            error('sine 타입은 주파수(Hz) 필요');
        end
        freq = varargin{1};
        dt   = 1e-3;
        t    = (0:dt:60)';
        mag  = magnitude * sin(2*pi*freq*t);

        switch lower(axis)
            case 'roll',  ts = timeseries([mag, zeros(size(t)), zeros(size(t))], t);
            case 'pitch', ts = timeseries([zeros(size(t)), mag, zeros(size(t))], t);
            case 'yaw',   ts = timeseries([zeros(size(t)), zeros(size(t)), mag], t);
        end
        ts.Name = 'tau_ext_ts';
        assignin('base', 'tau_ext_ts', ts);

        nonlinPath = [modelName '/Nonlinear'];
        try
            delete_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6');
            delete_block(blockPath);
        catch
        end

        add_block('simulink/Sources/From Workspace', ...
            [nonlinPath '/tau_ext_zero'], ...
            'Position',       [140, 243, 200, 263], ...
            'VariableName',   'tau_ext_ts',         ...
            'OutDataTypeStr', 'double',              ...
            'Interpolate',    'on');
        add_line(nonlinPath, 'tau_ext_zero/1', 'AC model/6', 'autorouting','on');
        save_system(modelName);
        fprintf('교란 설정: %s축 %.4f N*m, %.1fHz 정현파\n', axName, magnitude, freq);

    otherwise
        error('type은 constant/step/sine/off 중 하나');
end
end
