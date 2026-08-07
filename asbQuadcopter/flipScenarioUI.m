function flipScenarioUI()
%FLIPSCENARIOUI  Pick a disturbance scenario, dial in its strength, and fly it.
%
%   Two scenarios share one screen:
%     1) Impact  - hit the airframe at a chosen point, with a chosen impulse
%                  and direction. The hit applies a body-frame force and the
%                  matching torque r x F, so the drone is both shoved and spun.
%     2) Attitude - release the drone at altitude already tilted, and let it
%                  recover, re-hover, and fly back to its station.
%
%   Both drive the hooks in nonlinearAirframe (ImpactGen, MotorGate and the
%   init_* plant state) and leave the model, and the workspace, exactly as
%   they found them.
%
%   The hooks were deleted from the airframe in dd12f10, so if they are
%   missing this screen says so and points at restoreScenarioHooks('apply')
%   instead of quietly flying a plain hover and calling it a success.
%
%   Usage:  open the Quadcopter project, then run  flipScenarioUI
%
%   See also RESTORESCENARIOHOOKS.

MODEL   = 'asbQuadcopter';
AIRFR   = 'nonlinearAirframe';
ESTIM   = 'stateEstimator';
ARM     = 0.0624;      % Vehicle.Airframe.d - distance from CG to a motor [m]
HOVER_Z = 3.0;         % altitude the command profile parks at [m]
                       % (cmdData_hover3m.xlsx ramps to -3 m by t = 4 s and holds)

ui = struct();
ui.latching = false;
buildFigure();
showScenario('impact');
reportReadiness();

% ======================================================================
%  Layout
% ======================================================================
    function buildFigure()
        ui.fig = uifigure('Name','Flip Scenarios', 'Position',[100 80 1180 760]);
        outer = uigridlayout(ui.fig, [1 2]);
        outer.ColumnWidth = {400, '1x'};
        outer.Padding = [12 12 12 12];
        outer.ColumnSpacing = 14;

        % ---------------- left: controls ----------------
        left = uigridlayout(outer, [5 1]);
        left.RowHeight = {40, 76, '1x', 44, 26};
        left.Padding = [0 0 0 0];
        left.RowSpacing = 10;

        t = uilabel(left, 'Text','시나리오 선택', 'FontSize',20, 'FontWeight','bold');
        t.Layout.Row = 1;

        % mode cards
        modes = uigridlayout(left, [1 2]);
        modes.Padding = [0 0 0 0];
        modes.ColumnSpacing = 8;
        modes.Layout.Row = 2;
        ui.btnImpact = uibutton(modes, 'state', ...
            'Text', sprintf('① 타격\n기체를 친다'), 'FontSize',14, 'Value',true, ...
            'ValueChangedFcn', @(s,~) selectMode(s,'impact'));
        ui.btnAtt = uibutton(modes, 'state', ...
            'Text', sprintf('② 초기 자세\n기울여서 놓는다'), 'FontSize',14, 'Value',false, ...
            'ValueChangedFcn', @(s,~) selectMode(s,'attitude'));

        % parameter card holder
        ui.card = uipanel(left, 'BorderType','none');
        ui.card.Layout.Row = 3;
        cardGrid = uigridlayout(ui.card, [1 1]);
        cardGrid.Padding = [0 0 0 0];
        buildImpactPanel(cardGrid);
        buildAttitudePanel(cardGrid);

        ui.runBtn = uibutton(left, 'Text','▶  시뮬레이션 실행', 'FontSize',15, ...
            'FontWeight','bold', 'ButtonPushedFcn', @(~,~) onRun());
        ui.runBtn.Layout.Row = 4;

        ui.status = uilabel(left, 'Text','대기 중', 'FontColor',[0.35 0.35 0.35]);
        ui.status.Layout.Row = 5;

        % ---------------- right: results ----------------
        right = uigridlayout(outer, [2 1]);
        right.RowHeight = {'1x', 132};
        right.Padding = [0 0 0 0];
        right.RowSpacing = 10;

        plots = uigridlayout(right, [2 2]);
        plots.Layout.Row = 1;
        plots.Padding = [0 0 0 0];
        ui.axTilt = uiaxes(plots); title(ui.axTilt,'자세 기울기');   ylabel(ui.axTilt,'tilt [deg]');
        ui.axAlt  = uiaxes(plots); title(ui.axAlt,'고도');           ylabel(ui.axAlt,'alt [m]');
        ui.axXY   = uiaxes(plots); title(ui.axXY,'수평 궤적 (위에서 본 모습)');
        ui.axSon  = uiaxes(plots); title(ui.axSon,'소나 사용 구간');  ylabel(ui.axSon,'valid');
        for ax = [ui.axTilt ui.axAlt ui.axXY ui.axSon]
            grid(ax,'on'); xlabel(ax,'time [s]');
        end
        xlabel(ui.axXY,'X [m]'); ylabel(ui.axXY,'Y [m]');

        scorePanel = uipanel(right, 'Title','결과', 'FontWeight','bold');
        scorePanel.Layout.Row = 2;
        sg = uigridlayout(scorePanel, [1 2]);
        sg.ColumnWidth = {'1x', 190};
        ui.scoreTxt = uilabel(sg, 'Text','아직 실행하지 않았습니다.', ...
            'VerticalAlignment','top', 'Interpreter','none');
        ui.verdict = uilabel(sg, 'Text','', 'FontSize',26, 'FontWeight','bold', ...
            'HorizontalAlignment','center');
    end

    function buildImpactPanel(parent)
        ui.pImpact = uipanel(parent, 'Title','① 타격 설정', 'FontWeight','bold');
        ui.pImpact.Layout.Row = 1; ui.pImpact.Layout.Column = 1;
        g = uigridlayout(ui.pImpact, [9 2]);
        g.ColumnWidth = {150, '1x'};
        g.RowHeight = repmat({30}, 1, 9);

        ui.impJ    = addSlider(g, 1, '충격량 J [N·s]', [0.002 0.06], 0.030, '%.3f');
        ui.impRad  = addSlider(g, 2, '타격점 반경 [cm]', [0 ARM*100], 0, '%.1f');
        ui.impAzP  = addSlider(g, 3, '타격점 방위 [deg]', [0 360], 0, '%.0f');
        ui.impHgt  = addSlider(g, 4, '타격 높이 [cm]', [-5 5], 3, '%.1f');
        ui.impAzF  = addSlider(g, 5, '힘 방향 방위 [deg]', [0 360], 90, '%.0f');
        ui.impElF  = addSlider(g, 6, '힘 방향 앙각 [deg]', [-90 90], 0, '%.0f');
        ui.impDur  = addSlider(g, 7, '접촉 시간 [ms]', [20 100], 30, '%.0f');
        ui.impT0   = addSlider(g, 8, '타격 시각 [s]', [6 15], 10, '%.1f');

        hint = uilabel(g, 'Text', sprintf([ ...
            '· 기본값 = 무게중심 위쪽을 옆에서 미는 상황 → 롤로 넘어지며 옆으로 밀림\n' ...
            '· 이 형상 기준: J 0.03 → 약 58도 기울고 회복,  J 0.045 → 넘어가서 땅에 닿음\n' ...
            '· 높이 0 + 반경 0 = 정중앙 정타 → 안 돌고 밀리기만 함\n' ...
            '· 팔 끝을 수평으로 치면 토크가 대부분 요(yaw)로 간다. 요는 이 기체의\n' ...
            '  약한 축이라 작은 충격에도 크게 흔들린다 (flipYawDiag.m 참고)']), ...
            'FontSize',11, 'FontColor',[0.35 0.35 0.35]);
        hint.Layout.Row = 9; hint.Layout.Column = [1 2];
    end

    function buildAttitudePanel(parent)
        ui.pAtt = uipanel(parent, 'Title','② 초기 자세 설정', 'FontWeight','bold', 'Visible','off');
        ui.pAtt.Layout.Row = 1; ui.pAtt.Layout.Column = 1;
        g = uigridlayout(ui.pAtt, [7 2]);
        g.ColumnWidth = {150, '1x'};
        g.RowHeight = repmat({32}, 1, 7);

        ui.attTilt = addSlider(g, 1, '초기 기울기 [deg]', [0 180], 120, '%.0f', @() updateAdvice());
        ui.attAz   = addSlider(g, 2, '기울기 방향 [deg]', [0 360], 0, '%.0f');
        ui.attAlt  = addSlider(g, 3, '방출 고도 [m]', [2 25], 10, '%.1f', @() updateAdvice());
        ui.attSpin = addSlider(g, 4, '초기 회전 [deg/s]', [0 600], 0, '%.0f');
        ui.attHold = addSlider(g, 5, '모터 정지 시간 [s]', [0 1.5], 0, '%.2f');

        ui.advice = uilabel(g, 'Text','', 'FontSize',12, 'FontWeight','bold');
        ui.advice.Layout.Row = 6; ui.advice.Layout.Column = [1 2];

        hint = uilabel(g, 'Text', sprintf([ ...
            '· 모터 정지 시간 동안은 아이들 추력만 나가므로 사실상 자유낙하한다.\n' ...
            '  0 이면 컨트롤러가 t=0 부터 바로 싸운다 (기울기가 0.1초면 잡힌다).\n' ...
            '· 자세 회복 자체는 빠르다. 진짜 문제는 회복하는 동안 잃는 고도.']), ...
            'FontSize',11, 'FontColor',[0.35 0.35 0.35]);
        hint.Layout.Row = 7; hint.Layout.Column = [1 2];
        updateAdvice();
    end

    function h = addSlider(g, row, labelText, lims, val, fmt, varargin)
        extraCb = [];
        if ~isempty(varargin), extraCb = varargin{1}; end
        lab = uilabel(g, 'Text', labelText);
        lab.Layout.Row = row; lab.Layout.Column = 1;
        sub = uigridlayout(g, [1 2]);
        sub.Layout.Row = row; sub.Layout.Column = 2;
        sub.ColumnWidth = {'1x', 52};
        sub.Padding = [0 0 0 0]; sub.ColumnSpacing = 6;
        s = uislider(sub, 'Limits', lims, 'Value', val, ...
            'MajorTicks', [], 'MinorTicks', []);
        v = uilabel(sub, 'Text', sprintf(fmt, val), 'HorizontalAlignment','right');
        s.ValueChangedFcn  = @(src,~)  onSlide(src, v, fmt, extraCb);
        s.ValueChangingFcn = @(src,ev) set(v, 'Text', sprintf(fmt, ev.Value));
        h = s;
    end

    function onSlide(src, valLabel, fmt, extraCb)
        set(valLabel, 'Text', sprintf(fmt, src.Value));
        if ~isempty(extraCb), extraCb(); end
    end

% ======================================================================
%  Mode switching / advice
% ======================================================================
    function selectMode(src, mode)
        if ui.latching, return; end     % ignore the echo from latching the other button
        src.Value = true;               % keep the clicked one latched
        showScenario(mode);
    end

    function showScenario(mode)
        ui.mode = mode;
        isImpact = strcmp(mode,'impact');
        ui.pImpact.Visible = onOff(isImpact);
        ui.pAtt.Visible    = onOff(~isImpact);
        ui.latching = true;
        ui.btnImpact.Value = isImpact;
        ui.btnAtt.Value    = ~isImpact;
        ui.latching = false;
    end

    function s = onOff(tf)
        if tf, s = 'on'; else, s = 'off'; end
    end

    function updateAdvice()
        if ~isfield(ui,'attTilt'), return; end
        tilt = ui.attTilt.Value;
        need = recommendedAltitude(tilt);
        have = ui.attAlt.Value;
        if have >= need
            ui.advice.Text = sprintf('권장 최소 고도 %.1f m — 현재 %.1f m, 살아남을 만함', need, have);
            ui.advice.FontColor = [0.05 0.45 0.15];
        else
            ui.advice.Text = sprintf('권장 최소 고도 %.1f m — 현재 %.1f m, 땅에 닿을 수 있음', need, have);
            ui.advice.FontColor = [0.70 0.25 0.05];
        end
    end

    function h = recommendedAltitude(tiltDeg)
        % Rough guide fitted to the calibration sweep: 170 deg survives an 8 m
        % release with ~1.8 m to spare, 180 deg does not.
        h = 1.5 + 9.0 * (tiltDeg/180)^2.2;
    end

% ======================================================================
%  Readiness - the airframe hooks have to be there for any of this to mean
%  anything. Without them the model still runs; it just flies a plain hover
%  and every scenario scores a meaningless "성공".
% ======================================================================
    function reportReadiness()
        try
            st = restoreScenarioHooks('check');
        catch err
            setStatus(['훅 점검 실패: ' err.message], [0.75 0.15 0.10]);
            return;
        end
        if st.allReady
            setStatus(sprintf('준비 완료 — %s', gyroRegime()), [0.35 0.35 0.35]);
        else
            ui.runBtn.Enable = 'off';
            missing = {};
            if ~st.impact,    missing{end+1} = '타격'; end
            if ~st.motorGate, missing{end+1} = '모터 정지'; end
            if ~st.initState, missing{end+1} = '초기 자세'; end
            setStatus(sprintf('훅 없음(%s) — restoreScenarioHooks(''apply'') 먼저 실행', ...
                strjoin(missing,', ')), [0.75 0.15 0.10]);
            ui.scoreTxt.Text = sprintf([ ...
                'nonlinearAirframe 에 시나리오 훅이 없습니다 (커밋 dd12f10 에서 삭제됨).\n' ...
                '이 상태로 실행하면 외란이 하나도 없는 호버만 돌아 결과가 무의미합니다.\n\n' ...
                'MATLAB 명령창에서 한 번만 실행하세요:\n' ...
                '    restoreScenarioHooks(''apply'')\n\n' ...
                '그 뒤 이 창을 닫고 flipScenarioUI 를 다시 여세요.']);
            ui.verdict.Text = '';
        end
    end

    function s = gyroRegime()
        % unlockGyro=1 이면 자이로 포화가 풀린 상태라 결과 해석이 달라진다.
        try
            lim = max(evalin('base','Sensors.IMU.gyroLimits'));
        catch
            s = '자이로 설정 미확인'; return;
        end
        if lim > 100
            s = sprintf('자이로 포화 해제됨 (±%.0f rad/s, unlockGyro=1)', lim);
        else
            s = sprintf('자이로 실기값 (±%.1f rad/s)', lim);
        end
    end

    function setStatus(txt, col)
        ui.status.Text = txt;
        ui.status.FontColor = col;
        drawnow;
    end

% ======================================================================
%  Run
% ======================================================================
    function onRun()
        ui.runBtn.Enable = 'off';
        setStatus('실행 중...', [0.10 0.35 0.70]);
        cleanupObj = onCleanup(@() set(ui.runBtn,'Enable','on'));
        try
            cfg = gatherConfig();
            res = runScenario(cfg);
            drawResults(cfg, res);
            setStatus(sprintf('완료 (%.1f 초 시뮬) — %s', cfg.stopTime, gyroRegime()), ...
                [0.35 0.35 0.35]);
        catch err
            setStatus(['오류: ' err.message], [0.75 0.15 0.10]);
            rethrow(err);
        end
    end

    function cfg = gatherConfig()
        cfg = struct('mode', ui.mode, 'hoverZ', HOVER_Z);
        if strcmp(ui.mode,'impact')
            J    = ui.impJ.Value;
            dur  = ui.impDur.Value/1000;
            rad  = ui.impRad.Value/100;
            azP  = deg2rad(ui.impAzP.Value);
            hgt  = ui.impHgt.Value/100;
            azF  = deg2rad(ui.impAzF.Value);
            elF  = deg2rad(ui.impElF.Value);
            % Hit point on the airframe, body frame. Body z points down, so a
            % positive "height" above the CG is a negative z.
            cfg.r = [rad*cos(azP); rad*sin(azP); -hgt];
            % force direction, body frame: elevation 0 is horizontal, +90 pushes down
            n = [cos(elF)*cos(azF); cos(elF)*sin(azF); sin(elF)];
            cfg.F   = (J/dur) * n;
            cfg.dur = dur;
            cfg.t0  = ui.impT0.Value;
            cfg.J   = J;
            cfg.Cbe = eye(3);
            cfg.W   = zeros(3,1);
            cfg.xN  = [57; 95; -0.046];
            cfg.hold = 0;                           % motors live: it has to hover to be hit
            cfg.stopTime = max(20, cfg.t0 + 10);
            cfg.evalFrom = cfg.t0;
        else
            tilt = ui.attTilt.Value;
            az   = deg2rad(ui.attAz.Value);
            alt  = ui.attAlt.Value;
            spin = deg2rad(ui.attSpin.Value);
            axis = [-sin(az); cos(az); 0];          % horizontal, perpendicular to the tip direction
            cfg.Cbe = rodrigues(-deg2rad(tilt)*axis);
            cfg.W   = spin*axis;
            cfg.xN  = [57; 95; -alt];
            cfg.F   = zeros(3,1);
            cfg.r   = zeros(3,1);
            cfg.dur = 0.03; cfg.t0 = 1e6;           % no impact
            cfg.hold = ui.attHold.Value;            % motors dead for this long
            cfg.tilt0 = tilt; cfg.alt0 = alt;
            cfg.stopTime = 25;
            cfg.evalFrom = 0;
        end
    end

    function res = runScenario(cfg)
        load_system(MODEL); load_system(AIRFR);

        % --- push scenario parameters, remembering what was there ---
        names = {'init_Cbe','init_W','init_xN','init_vN', ...
                 'impact_t0','impact_dur','impact_F','impact_r','release_delay'};
        vals  = {cfg.Cbe, cfg.W, cfg.xN, zeros(3,1), ...
                 cfg.t0, cfg.dur, cfg.F, cfg.r, cfg.hold};
        prior = struct('name',{},'value',{},'existed',{});
        for k = 1:numel(names)
            prior(k).name    = names{k};
            prior(k).existed = evalin('base', sprintf('exist(''%s'',''var'')==1', names{k}));
            if prior(k).existed
                prior(k).value = evalin('base', names{k});
            end
            assignin('base', names{k}, vals{k});
        end
        restoreWs = onCleanup(@() restoreWorkspace(prior));   %#ok<NASGU>
        assignin('base','g', 9.81);

        % --- silence anything that would fight the scenario, then restore ---
        NL = [AIRFR '/Nonlinear'];
        sA = [NL '/Step']; sB = [NL '/Step1'];
        saved.A  = get_param(sA,'After');
        saved.B  = get_param(sB,'After');
        saved.sf = get_param(MODEL,'StopFcn');
        saved.st = get_param(MODEL,'StopTime');
        saved.sl = get_param(MODEL,'SignalLogging');
        saved.pc = get_param(MODEL,'EnablePacing');
        restore = onCleanup(@() restoreModel(sA, sB, saved));  %#ok<NASGU>
        set_param(sA,'After','[0 0 0]');
        set_param(sB,'After','[0 0 0]');
        set_param(MODEL,'StopFcn','');
        set_param(MODEL,'StopTime', num2str(cfg.stopTime));
        set_param(MODEL,'SignalLogging','on','SignalLoggingName','logsout');
        set_param(MODEL,'EnablePacing','off');   % otherwise a 25 s run takes 25 s

        gatePort = markSonarGate();                 % [] if the gate block is absent
        unmark   = onCleanup(@() unmarkSonarGate(gatePort));   %#ok<NASGU>

        out = sim(MODEL, 'ReturnWorkspaceOutputs','on');

        % --- extract ---
        % States comes from the log_states To Workspace block, the same source
        % every other script here uses. It is NOT in logsout: no signal in the
        % top model is marked for logging, so logsout only ever holds what
        % markSonarGate turns on below.
        ls = out.log_states;
        res.t = ls.X_ned.Time;
        X = asNx3(ls.X_ned.Data);
        res.X = X(:,1);
        res.Y = X(:,2);
        res.Z = -X(:,3);
        R33 = squeeze(double(ls.DCM_be.Data(3,3,:)));
        res.tilt = acosd(min(max(R33, -1), 1));

        % --- sonar validity window ---
        % Reproduce the estimator's own test from the true states, so the panel
        % works whether or not the estimator signal happens to be logged:
        %   slant = min(alt / max(R33, 0.087), 5)      flightControlSystem/SonarTilt_Distort
        %   valid = R33 > cos(45 deg) and 0.44 <= slant <= 5   .../SonarRangeCheck
        slant = min(res.Z ./ max(R33, 0.087), 5);
        res.gate    = double((R33 > 0.7071) & (slant >= 0.44) & (slant <= 5.0));
        res.gateSrc = '참값으로 재현';
        try
            gv = out.get('logsout').getElement('meas_valid').Values;
            res.gate    = interp1(gv.Time, double(squeeze(gv.Data)), res.t, 'previous','extrap');
            res.gateSrc = '추정기 실측';
        catch
        end

        % --- metrics ---
        w = res.t >= cfg.evalFrom;
        res.maxTilt  = max(res.tilt(w));
        res.minAlt   = min(res.Z(w));
        res.crashed  = res.minAlt <= 0.06;
        res.target   = [cfg.xN(1); cfg.xN(2); cfg.hoverZ];
        res.errXY    = hypot(res.X(end)-res.target(1), res.Y(end)-res.target(2));
        res.errZ     = res.Z(end) - res.target(3);
        res.endTilt  = res.tilt(end);
        res.maxDev   = max(hypot(res.X(w)-res.target(1), res.Y(w)-res.target(2)));

        % Settling is only meaningful after the upset, so start looking at the
        % peak tilt rather than at the moment the scenario begins.
        wIdx = find(w);
        [~, rel] = max(res.tilt(wIdx));
        iPeak = wIdx(rel);
        rest = find(res.tilt(iPeak:end) < 5, 1);
        if isempty(rest)
            res.settle = NaN;
        else
            res.settle = res.t(iPeak - 1 + rest) - res.t(iPeak);
        end

        res.gateFrac = mean(res.gate);

        res.success = ~res.crashed && res.endTilt < 10 && res.errXY < 1.0 && abs(res.errZ) < 0.5;
    end

    function M = asNx3(D)
        % log_states hands a 3-vector back as N-by-3 or as 3-by-1-by-N
        % depending on the signal; normalise to N-by-3.
        D = double(D);
        if ndims(D) == 3
            M = squeeze(D).';
        elseif size(D,1) == 3 && size(D,2) ~= 3
            M = D.';
        else
            M = D;
        end
    end

    function restoreWorkspace(prior)
        for k = 1:numel(prior)
            if prior(k).existed
                assignin('base', prior(k).name, prior(k).value);
            else
                evalin('base', sprintf('clear %s', prior(k).name));
            end
        end
    end

    function restoreModel(sA, sB, saved)
        set_param(sA,'After',saved.A);
        set_param(sB,'After',saved.B);
        set_param(MODEL,'StopFcn',saved.sf);
        set_param(MODEL,'StopTime',saved.st);
        set_param(MODEL,'SignalLogging',saved.sl);
        set_param(MODEL,'EnablePacing',saved.pc);
    end

    function port = markSonarGate()
        port = [];
        try
            load_system(ESTIM);
            ea = [ESTIM '/State Estimator/EstimatorAltitude'];
            ph = get_param([ea '/SonarRangeCheck'],'PortHandles');
            port = ph.Outport(2);
            set_param(port,'DataLogging','on', ...
                'DataLoggingNameMode','Custom','DataLoggingName','meas_valid');
        catch
            port = [];
        end
    end

    function unmarkSonarGate(port)
        if isempty(port), return; end
        try
            set_param(port,'DataLogging','off');
        catch
        end
    end

% ======================================================================
%  Results
% ======================================================================
    function drawResults(cfg, res)
        cla(ui.axTilt); cla(ui.axAlt); cla(ui.axXY); cla(ui.axSon);

        plot(ui.axTilt, res.t, res.tilt, 'LineWidth',1.4, 'Color',[0.45 0.10 0.55]);
        yline(ui.axTilt, 45, '--', '소나 게이트 45°');
        ylim(ui.axTilt, [0 max(50, max(res.tilt)*1.1)]);

        plot(ui.axAlt, res.t, res.Z, 'LineWidth',1.5, 'Color',[0.10 0.10 0.10]); hold(ui.axAlt,'on');
        yline(ui.axAlt, cfg.hoverZ, '--', '목표 고도');
        yline(ui.axAlt, 0, '-', '지면', 'Color',[0.7 0.2 0.1]);
        hold(ui.axAlt,'off');

        plot(ui.axXY, res.X, res.Y, 'LineWidth',1.3, 'Color',[0.10 0.35 0.70]); hold(ui.axXY,'on');
        plot(ui.axXY, res.target(1), res.target(2), 'p', 'MarkerSize',13, ...
            'MarkerFaceColor',[0.95 0.75 0.10], 'MarkerEdgeColor','k');
        plot(ui.axXY, res.X(end), res.Y(end), 'o', 'MarkerSize',8, ...
            'MarkerFaceColor',[0.85 0.25 0.15], 'MarkerEdgeColor','k');
        legend(ui.axXY, {'궤적','목표','최종'}, 'Location','best', 'FontSize',8);
        axis(ui.axXY,'equal'); hold(ui.axXY,'off');

        area(ui.axSon, res.t, res.gate, 'FaceColor',[0.72 0.85 0.98], 'EdgeColor','none');
        ylim(ui.axSon, [-0.1 1.1]);
        title(ui.axSon, sprintf('소나 사용 구간 (%s)', res.gateSrc));

        lines = {};
        if strcmp(cfg.mode,'impact')
            lines{end+1} = sprintf('타격  : J = %.3f N·s,  접촉 %.0f ms,  t = %.1f s', ...
                cfg.J, cfg.dur*1000, cfg.t0);
            lines{end+1} = sprintf('타격점: r = [%.1f, %.1f, %.1f] cm (몸체)', cfg.r*100);
        else
            lines{end+1} = sprintf('초기  : 기울기 %.0f°,  방출 고도 %.1f m,  모터 정지 %.2f s', ...
                cfg.tilt0, cfg.alt0, cfg.hold);
        end
        lines{end+1} = sprintf('최대 기울기 : %.1f°', res.maxTilt);
        if isnan(res.settle)
            lines{end+1} = '자세 회복   : 시간 내 5° 미만 도달 못 함';
        else
            lines{end+1} = sprintf('자세 회복   : %.2f 초 (5° 미만)', res.settle);
        end
        lines{end+1} = sprintf('최저 고도   : %.2f m%s', res.minAlt, tern(res.crashed,'   ← 지면 접촉',''));
        lines{end+1} = sprintf('최대 이탈   : 수평 %.3f m (목표점 기준)', res.maxDev);
        lines{end+1} = sprintf('복귀 오차   : 수평 %.3f m,  고도 %+.3f m,  최종 기울기 %.1f°', ...
            res.errXY, res.errZ, res.endTilt);
        lines{end+1} = sprintf('소나 사용   : %.0f%% (나머지는 기압계가 대신함)', res.gateFrac*100);
        ui.scoreTxt.Text = strjoin(lines, newline);

        if res.success
            ui.verdict.Text = '성공';
            ui.verdict.FontColor = [0.05 0.50 0.18];
        else
            ui.verdict.Text = '실패';
            ui.verdict.FontColor = [0.80 0.15 0.10];
        end
    end

    function s = tern(c, a, b)
        if c, s = a; else, s = b; end
    end

    function R = rodrigues(phi)
        th = norm(phi);
        K = [0 -phi(3) phi(2); phi(3) 0 -phi(1); -phi(2) phi(1) 0];
        if th < 1e-9
            R = eye(3) + K + 0.5*(K*K);
        else
            R = eye(3) + (sin(th)/th)*K + ((1-cos(th))/th^2)*(K*K);
        end
    end
end
