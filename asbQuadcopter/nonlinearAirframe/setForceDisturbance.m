% setForceDisturbance.m
% 시뮬레이션 전에 병진 외란 신호를 설정합니다.
%
% 사용법:
%   setForceDisturbance('constant', 'x', 0.01)
%   setForceDisturbance('step',     'y', 0.01, 5, 10)
%   setForceDisturbance('sine',     'x', 0.01, 2)        % 2Hz
%   setForceDisturbance('random',   'z', 0.01)           % 가우시안
%   setForceDisturbance('random',   'x', 0.01, 10)       % seed 지정
%   setForceDisturbance('off')
%
% axis: 'x' (전진), 'y' (측면), 'z' (수직)
% magnitude 단위: N (뉴턴)

function setForceDisturbance(type, axis, magnitude, varargin)

modelName = 'nonlinearAirframe';
blockPath = [modelName '/Nonlinear/force_ext_zero'];

if strcmp(type, 'off')
    nonlinPath = [modelName '/Nonlinear'];
    blockType  = get_param(blockPath, 'BlockType');
    if ~strcmp(blockType, 'Constant')
        delete_line(nonlinPath, 'force_ext_zero/1', 'AC model/7');
        delete_block(blockPath);
        add_block('simulink/Sources/Constant', ...
            [nonlinPath '/force_ext_zero'], ...
            'Position', [140, 273, 200, 293], ...
            'Value',    '[0;0;0]');
        add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');
        save_system(modelName);
    else
        set_param(blockPath, 'Value', '[0;0;0]');
    end
    fprintf('힘 교란 OFF\n');
    return;
end

switch lower(axis)
    case 'x', axIdx = 1; axName = 'X(전진)';
    case 'y', axIdx = 2; axName = 'Y(측면)';
    case 'z', axIdx = 3; axName = 'Z(수직)';
    otherwise, error('axis는 x/y/z 중 하나');
end

switch lower(type)
    case 'constant'
        nonlinPath = [modelName '/Nonlinear'];
        blockType  = get_param(blockPath, 'BlockType');
        if ~strcmp(blockType, 'Constant')
            delete_line(nonlinPath, 'force_ext_zero/1', 'AC model/7');
            delete_block(blockPath);
            add_block('simulink/Sources/Constant', ...
                [nonlinPath '/force_ext_zero'], ...
                'Position',       [140, 273, 200, 293], ...
                'Value',          '[0;0;0]',             ...
                'OutDataTypeStr', 'double');
            add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');
        end
        v = [0; 0; 0]; v(axIdx) = magnitude;
        set_param(blockPath, 'Value', sprintf('[%f;%f;%f]', v(1), v(2), v(3)));
        save_system(modelName);
        fprintf('힘 교란: %s축 %.4f N 상수\n', axName, magnitude);

    case 'step'
        if numel(varargin) < 2
            error('step 타입은 t_start, t_end 필요');
        end
        t_start = varargin{1};
        t_end   = varargin{2};
        t   = [0, t_start-1e-6, t_start, t_end, t_end+1e-6, 1000];
        mag = [0, 0, magnitude, magnitude, 0, 0];
        data = zeros(6, 3);
        data(:, axIdx) = mag';
        ts = timeseries(data, t');
        ts.Name = 'force_ext_ts';
        assignin('base', 'force_ext_ts', ts);

        nonlinPath = [modelName '/Nonlinear'];
        try
            delete_line(nonlinPath, 'force_ext_zero/1', 'AC model/7');
            delete_block(blockPath);
        catch
        end
        add_block('simulink/Sources/From Workspace', ...
            [nonlinPath '/force_ext_zero'], ...
            'Position',       [140, 273, 200, 293], ...
            'VariableName',   'force_ext_ts',        ...
            'OutDataTypeStr', 'double',               ...
            'Interpolate',    'on');
        add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');
        save_system(modelName);
        fprintf('힘 교란: %s축 %.4f N, t=%.1f~%.1fs\n', axName, magnitude, t_start, t_end);

    case 'sine'
        if numel(varargin) < 1
            error('sine 타입은 주파수(Hz) 필요');
        end
        freq = varargin{1};
        dt   = 1e-3;
        t    = (0:dt:60)';
        mag  = magnitude * sin(2*pi*freq*t);
        data = zeros(length(t), 3);
        data(:, axIdx) = mag;
        ts = timeseries(data, t);
        ts.Name = 'force_ext_ts';
        assignin('base', 'force_ext_ts', ts);

        nonlinPath = [modelName '/Nonlinear'];
        try
            delete_line(nonlinPath, 'force_ext_zero/1', 'AC model/7');
            delete_block(blockPath);
        catch
        end
        add_block('simulink/Sources/From Workspace', ...
            [nonlinPath '/force_ext_zero'], ...
            'Position',       [140, 273, 200, 293], ...
            'VariableName',   'force_ext_ts',        ...
            'OutDataTypeStr', 'double',               ...
            'Interpolate',    'on');
        add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');
        save_system(modelName);
        fprintf('힘 교란: %s축 %.4f N, %.1fHz 정현파\n', axName, magnitude, freq);

    case 'random'
        % 대역제한 가우시안 노이즈 (5Hz LPF)
        % varargin{1} = seed (선택), varargin{2} = cutoff Hz (기본 5Hz)
        if numel(varargin) >= 1
            rng(varargin{1});
        end
        cutoff_hz = 5;
        if numel(varargin) >= 2
            cutoff_hz = varargin{2};
        end
        dt   = 1e-3;
        t    = (0:dt:60)';
        raw  = magnitude * randn(size(t));
        fc   = cutoff_hz / (1/(2*dt));
        [b_f, a_f] = butter(1, fc);
        mag  = filtfilt(b_f, a_f, raw);
        data = zeros(length(t), 3);
        data(:, axIdx) = mag;
        ts = timeseries(data, t);
        ts.Name = 'force_ext_ts';
        assignin('base', 'force_ext_ts', ts);

        nonlinPath = [modelName '/Nonlinear'];
        try
            delete_line(nonlinPath, 'force_ext_zero/1', 'AC model/7');
            delete_block(blockPath);
        catch
        end
        add_block('simulink/Sources/From Workspace', ...
            [nonlinPath '/force_ext_zero'], ...
            'Position',       [140, 273, 200, 293], ...
            'VariableName',   'force_ext_ts',        ...
            'OutDataTypeStr', 'double',               ...
            'Interpolate',    'on');
        add_line(nonlinPath, 'force_ext_zero/1', 'AC model/7', 'autorouting', 'on');
        save_system(modelName);
        fprintf('힘 교란: %s축 %.4f N 가우시안 노이즈\n', axName, magnitude);

    otherwise
        error('type은 constant/step/sine/random/off 중 하나');
end
end
