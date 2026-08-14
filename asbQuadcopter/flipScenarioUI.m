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
%   missing this screen locks the run button instead of quietly flying a
%   plain hover and calling it a success. The [Apply hooks (session only)] button
%   puts them back with restoreScenarioHooks('session'), which patches the
%   loaded model only and leaves nonlinearAirframe.slx untouched on disk -
%   no diff on the shared binary model, no merge conflict with the team.
%
%   Metrics stop at the first ground contact. The plant only pins z and
%   kills downward velocity when it lands, so a crashed airframe keeps its
%   horizontal speed and slides for hundreds of metres; scoring that tail
%   produced nonsense like "maxDev 2180 m".
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

        t = uilabel(left, 'Text','Choose a scenario', 'FontSize',20, 'FontWeight','bold');
        t.Layout.Row = 1;

        % mode cards
        modes = uigridlayout(left, [1 2]);
        modes.Padding = [0 0 0 0];
        modes.ColumnSpacing = 8;
        modes.Layout.Row = 2;
        ui.btnImpact = uibutton(modes, 'state', ...
            'Text', sprintf('(1) Impact\nhit the airframe'), 'FontSize',14, 'Value',true, ...
            'ValueChangedFcn', @(s,~) selectMode(s,'impact'));
        ui.btnAtt = uibutton(modes, 'state', ...
            'Text', sprintf('(2) Initial attitude\nrelease it tilted'), 'FontSize',14, 'Value',false, ...
            'ValueChangedFcn', @(s,~) selectMode(s,'attitude'));

        % parameter card holder
        ui.card = uipanel(left, 'BorderType','none');
        ui.card.Layout.Row = 3;
        cardGrid = uigridlayout(ui.card, [1 1]);
        cardGrid.Padding = [0 0 0 0];
        buildImpactPanel(cardGrid);
        buildAttitudePanel(cardGrid);

        actions = uigridlayout(left, [1 2]);
        actions.Layout.Row = 4;
        actions.Padding = [0 0 0 0];
        actions.ColumnWidth = {170, '1x'};
        actions.ColumnSpacing = 8;
        ui.hookBtn = uibutton(actions, 'Text','Apply hooks (session only)', 'FontSize',13, ...
            'Enable','off', 'ButtonPushedFcn', @(~,~) onApplyHooks());
        ui.runBtn = uibutton(actions, 'Text','>  Run simulation', 'FontSize',15, ...
            'FontWeight','bold', 'ButtonPushedFcn', @(~,~) onRun());

        ui.status = uilabel(left, 'Text','Idle', 'FontColor',[0.35 0.35 0.35]);
        ui.status.Layout.Row = 5;

        % ---------------- right: results ----------------
        right = uigridlayout(outer, [2 1]);
        right.RowHeight = {'1x', 132};
        right.Padding = [0 0 0 0];
        right.RowSpacing = 10;

        plots = uigridlayout(right, [2 2]);
        plots.Layout.Row = 1;
        plots.Padding = [0 0 0 0];
        ui.axTilt = uiaxes(plots); title(ui.axTilt,'Attitude tilt');  ylabel(ui.axTilt,'tilt [deg]');
        ui.axAlt  = uiaxes(plots); title(ui.axAlt,'Altitude');        ylabel(ui.axAlt,'alt [m]');
        ui.axXY   = uiaxes(plots); title(ui.axXY,'Horizontal track (top view)');
        ui.axSon  = uiaxes(plots); title(ui.axSon,'Sonar valid window'); ylabel(ui.axSon,'valid');
        for ax = [ui.axTilt ui.axAlt ui.axXY ui.axSon]
            grid(ax,'on'); xlabel(ax,'time [s]');
        end
        xlabel(ui.axXY,'X [m]'); ylabel(ui.axXY,'Y [m]');

        scorePanel = uipanel(right, 'Title','Results', 'FontWeight','bold');
        scorePanel.Layout.Row = 2;
        sg = uigridlayout(scorePanel, [1 2]);
        sg.ColumnWidth = {'1x', 190};
        ui.scoreTxt = uilabel(sg, 'Text','Nothing has been run yet.', ...
            'VerticalAlignment','top', 'Interpreter','none');
        ui.verdict = uilabel(sg, 'Text','', 'FontSize',26, 'FontWeight','bold', ...
            'HorizontalAlignment','center');
    end

    function buildImpactPanel(parent)
        ui.pImpact = uipanel(parent, 'Title','(1) Impact settings', 'FontWeight','bold');
        ui.pImpact.Layout.Row = 1; ui.pImpact.Layout.Column = 1;
        g = uigridlayout(ui.pImpact, [9 2]);
        g.ColumnWidth = {150, '1x'};
        g.RowHeight = [repmat({30}, 1, 8), {'1x'}];   % last row = hint, keep it from clipping

        ui.impJ    = addSlider(g, 1, 'Impulse J [N·s]', [0.002 0.06], 0.030, '%.3f');
        ui.impRad  = addSlider(g, 2, 'Hit radius [cm]', [0 ARM*100], 0, '%.1f');
        ui.impAzP  = addSlider(g, 3, 'Hit azimuth [deg]', [0 360], 0, '%.0f');
        ui.impHgt  = addSlider(g, 4, 'Hit height [cm]', [-5 5], 3, '%.1f');
        ui.impAzF  = addSlider(g, 5, 'Force azimuth [deg]', [0 360], 90, '%.0f');
        ui.impElF  = addSlider(g, 6, 'Force elevation [deg]', [-90 90], 0, '%.0f');
        ui.impDur  = addSlider(g, 7, 'Contact time [ms]', [20 100], 30, '%.0f');
        ui.impT0   = addSlider(g, 8, 'Hit time [s]', [6 15], 10, '%.1f');

        hint = uilabel(g, 'Text', sprintf([ ...
            '- Default = pushed from the side, 3 cm above the CG -> rolls and slides.\n' ...
            '- Measured on this geometry (2026-08-07, unlockGyro=1) peak tilt / recovery:\n' ...
            '    J 0.015 -> 5.8 deg, 0.07 s     J 0.022 -> 9.9 deg, 0.18 s\n' ...
            '    J 0.030 -> 13.3 deg, 0.27 s    J 0.045 -> 20.0 deg, 0.75 s\n' ...
            '  All recover, none touch the ground. Even the slider max (0.06) will not tip it.\n' ...
            '  Below J 0.010 the response is buried in the hover tilt jitter (about 4 deg).\n' ...
            '- Height 0 + radius 0 = dead-centre hit -> it translates without rotating.\n' ...
            '- Hitting an arm tip horizontally sends most of the torque into yaw, the weak\n' ...
            '  axis of this airframe, so it swings hard for very little input (flipYawDiag.m).']), ...
            'FontSize',11, 'FontColor',[0.35 0.35 0.35]);
        hint.Layout.Row = 9; hint.Layout.Column = [1 2];
    end

    function buildAttitudePanel(parent)
        ui.pAtt = uipanel(parent, 'Title','(2) Initial attitude settings', 'FontWeight','bold', 'Visible','off');
        ui.pAtt.Layout.Row = 1; ui.pAtt.Layout.Column = 1;
        g = uigridlayout(ui.pAtt, [7 2]);
        g.ColumnWidth = {150, '1x'};
        g.RowHeight = [repmat({32}, 1, 6), {'1x'}];   % last row = hint, keep it from clipping

        ui.attTilt = addSlider(g, 1, 'Initial tilt [deg]', [0 180], 120, '%.0f', @() updateAdvice());
        ui.attAz   = addSlider(g, 2, 'Tilt azimuth [deg]', [0 360], 0, '%.0f');
        ui.attAlt  = addSlider(g, 3, 'Release altitude [m]', [2 25], 10, '%.1f', @() updateAdvice());
        ui.attSpin = addSlider(g, 4, 'Initial spin [deg/s]', [0 600], 0, '%.0f');
        ui.attHold = addSlider(g, 5, 'Motor hold [s]', [0 1.5], 0, '%.2f');

        ui.advice = uilabel(g, 'Text','', 'FontSize',12, 'FontWeight','bold');
        ui.advice.Layout.Row = 6; ui.advice.Layout.Column = [1 2];

        hint = uilabel(g, 'Text', sprintf([ ...
            '- During the motor hold only idle thrust is commanded, so it is effectively in\n' ...
            '  free fall. At 0 the controller fights from t=0.\n' ...
            '- Measured 2026-08-07 (tilt azimuth 0 deg, no spin, no hold):\n' ...
            '    3 m / 0 deg -> peak 37 deg, recovers in 1.33 s, 1.5 m final error (survives)\n' ...
            '    4 m / 0 deg -> ground    6 m / 0 deg -> ground    8 m / 0 deg -> ground\n' ...
            '    4 m / 30, 60, 90, 120 deg -> ground, every one\n' ...
            '- So the outcome is decided by release altitude, not tilt. Above the 3 m hover\n' ...
            '  target the current controller cannot arrest the fall. Attitude recovery itself\n' ...
            '  is fast; the real problem is the altitude lost while recovering.\n' ...
            '- Also suspect the estimator initial condition: it starts from the pad (alt 0),\n' ...
            '  so it carries an altitude error from the moment of release.']), ...
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
        % The old version quoted a "recommended minimum altitude" that grew with
        % tilt - i.e. it said dropping from higher was safer. The sweep says the
        % opposite: altitude decides the outcome and more of it is strictly
        % worse. 0 deg survives at 3 m and hits the ground at 4, 6 and 8 m, and
        % at 4 m every tilt from 30 to 120 deg hits the ground too. So advise on
        % altitude alone, and do not promise survival we have not measured.
        if ~isfield(ui,'attAlt'), return; end
        alt = ui.attAlt.Value;
        if alt <= HOVER_Z + 0.2
            ui.advice.Text = sprintf( ...
                'Release at %.1f m - near the hover target (%.1f m). The only band that survives.', ...
                alt, HOVER_Z);
            ui.advice.FontColor = [0.05 0.45 0.15];
        else
            ui.advice.Text = sprintf( ...
                'Release at %.1f m - above the target (%.1f m). Almost always crashes, whatever the tilt.', ...
                alt, HOVER_Z);
            ui.advice.FontColor = [0.70 0.25 0.05];
        end
    end

% ======================================================================
%  Readiness - the airframe hooks have to be there for any of this to mean
%  anything. Without them the model still runs; it just flies a plain hover
%  and every scenario scores a meaningless "SUCCESS".
% ======================================================================
    function reportReadiness()
        try
            st = restoreScenarioHooks('check');
        catch err
            % If the check itself blows up we cannot tell whether the hooks are
            % there. Running blind would score a disturbance-free hover as a
            % "success", so lock the run button instead.
            ui.runBtn.Enable  = 'off';
            ui.hookBtn.Enable = 'off';
            setStatus(['Hook check failed: ' err.message], [0.75 0.15 0.10]);
            ui.scoreTxt.Text = sprintf([ ...
                'restoreScenarioHooks(''check'') failed, so the presence of the scenario\n' ...
                'hooks cannot be confirmed. Running unchecked would score a\n' ...
                'disturbance-free hover as a success, so the run button is locked.\n\nError: %s'], err.message);
            ui.verdict.Text = '';
            return;
        end
        if st.allReady
            ui.runBtn.Enable  = 'on';
            ui.hookBtn.Enable = 'off';
            ui.hookBtn.Text   = 'Hooks applied';
            ui.scoreTxt.Text  = 'Nothing has been run yet.';
            ui.verdict.Text   = '';
            setStatus(sprintf('Ready - %s', gyroRegime()), [0.35 0.35 0.35]);
        else
            ui.runBtn.Enable  = 'off';
            ui.hookBtn.Enable = 'on';
            missing = {};
            if ~st.impact,    missing{end+1} = 'impact'; end
            if ~st.motorGate, missing{end+1} = 'motor gate'; end
            if ~st.initState, missing{end+1} = 'initial attitude'; end
            if ~st.clock,     missing{end+1} = 'discrete clock'; end
            setStatus(sprintf('Hooks missing (%s) - press [Apply hooks] on the left', ...
                strjoin(missing,', ')), [0.75 0.15 0.10]);
            ui.scoreTxt.Text = sprintf([ ...
                'nonlinearAirframe has no scenario hooks (removed in commit dd12f10).\n' ...
                'Running like this would fly a hover with no disturbance at all, which\n' ...
                'is meaningless, so the run button is locked.\n\n' ...
                '[Apply hooks (session only)] = restoreScenarioHooks(''session'')\n' ...
                '  -> Patches the loaded model only. The nonlinearAirframe.slx file on\n' ...
                '     disk is untouched, so no diff lands on the shared model.\n' ...
                '     It is lost when MATLAB closes - press it again each session.\n\n' ...
                'To restore it on disk for good, run restoreScenarioHooks(''apply'') from\n' ...
                'the command window - but that does modify the shared nonlinearAirframe.slx.']);
            ui.verdict.Text = '';
        end
    end

    function onApplyHooks()
        ui.hookBtn.Enable = 'off';
        setStatus('Applying hooks...', [0.10 0.35 0.70]);
        try
            restoreScenarioHooks('session');
        catch err
            setStatus(['Applying hooks failed: ' err.message], [0.75 0.15 0.10]);
            ui.hookBtn.Enable = 'on';
            return;
        end
        reportReadiness();
    end

    function s = gyroRegime()
        % With unlockGyro=1 the gyro no longer saturates, which changes how every
        % number on screen has to be read.
        try
            lim = max(evalin('base','Sensors.IMU.gyroLimits'));
        catch
            s = 'gyro setting unknown'; return;
        end
        if lim > 100
            s = sprintf('gyro saturation OFF (+/-%.0f rad/s, unlockGyro=1)', lim);
        else
            s = sprintf('real gyro (+/-%.1f rad/s)', lim);
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
        setStatus('Running...', [0.10 0.35 0.70]);
        cleanupObj = onCleanup(@() set(ui.runBtn,'Enable','on'));
        try
            cfg = gatherConfig();
            res = runScenario(cfg);
            drawResults(cfg, res);
            setStatus(sprintf('Done (%.1f s simulated) - %s', cfg.stopTime, gyroRegime()), ...
                [0.35 0.35 0.35]);
        catch err
            setStatus(['Error: ' err.message], [0.75 0.15 0.10]);
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

        gate   = markSonarGate();                   % .port = [] if the block is absent
        unmark = onCleanup(@() unmarkSonarGate(gate));   %#ok<NASGU>

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
        res.gateSrc = 'reconstructed from truth';
        try
            gv = out.get('logsout').getElement('meas_valid').Values;
            res.gate    = interp1(gv.Time, double(squeeze(gv.Data)), res.t, 'previous','extrap');
            res.gateSrc = 'logged from estimator';
        catch
        end

        % --- metrics ---
        % Everything after ground contact is noise. SO3_dynamics models the
        % ground by pinning z to ground_z and killing downward velocity only:
        % horizontal speed survives and the controller keeps commanding thrust,
        % so a crashed airframe slides hundreds of metres and reports things
        % like "maxDev 2180 m". Stop the evaluation window at the first touch.
        GROUND = 0.06;                 % ground_z = -0.046 m, plus a little slack
        w    = res.t >= cfg.evalFrom;
        iHit = find(w & (res.Z <= GROUND), 1);
        res.crashed = ~isempty(iHit);
        if res.crashed
            res.tHit = res.t(iHit);
            w = w & (res.t <= res.tHit);
        else
            res.tHit = NaN;
        end
        % Already on the ground when the scenario was supposed to start: the
        % baseline flight failed on its own and nothing below means anything.
        res.preCrashed = res.crashed && (res.tHit <= cfg.evalFrom + 1e-9);
        wIdx = find(w);
        if isempty(wIdx)
            error('flipScenarioUI:emptyWindow', ...
                'Evaluation window is empty (start %.2f s > run length %.2f s).', ...
                cfg.evalFrom, res.t(end));
        end
        iEnd  = wIdx(end);
        res.w = w;  res.iEnd = iEnd;

        res.target   = [cfg.xN(1); cfg.xN(2); cfg.hoverZ];
        res.maxTilt  = max(res.tilt(w));
        res.minAlt   = min(res.Z(w));
        res.errXY    = hypot(res.X(iEnd)-res.target(1), res.Y(iEnd)-res.target(2));
        res.errZ     = res.Z(iEnd) - res.target(3);
        res.endTilt  = res.tilt(iEnd);
        res.maxDev   = max(hypot(res.X(w)-res.target(1), res.Y(w)-res.target(2)));
        res.tEval    = res.t(iEnd);

        % Settling is only meaningful after the upset, so start looking at the
        % peak tilt rather than at the moment the scenario begins.
        [~, rel] = max(res.tilt(wIdx));
        iPeak = wIdx(rel);
        rest = find(res.tilt(iPeak:iEnd) < 5, 1);
        if isempty(rest)
            res.settle = NaN;
        else
            res.settle = res.t(iPeak - 1 + rest) - res.t(iPeak);
        end

        res.gateFrac = mean(res.gate(w));

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

    function g = markSonarGate()
        % Switching port logging on marks stateEstimator dirty, and that model
        % is shared with the rest of the team. Remember the flag so a run does
        % not leave "unsaved changes" sitting on someone else's file.
        g = struct('port', [], 'dirty', '');
        try
            load_system(ESTIM);
            g.dirty = get_param(ESTIM,'Dirty');
            ea = [ESTIM '/State Estimator/EstimatorAltitude'];
            ph = get_param([ea '/SonarRangeCheck'],'PortHandles');
            g.port = ph.Outport(2);
            set_param(g.port,'DataLogging','on', ...
                'DataLoggingNameMode','Custom','DataLoggingName','meas_valid');
        catch
            g.port = [];
        end
    end

    function unmarkSonarGate(g)
        if isempty(g.port), return; end
        try
            set_param(g.port,'DataLogging','off');
            set_param(ESTIM,'Dirty',g.dirty);
        catch
        end
    end

% ======================================================================
%  Results
% ======================================================================
    function drawResults(cfg, res)
        cla(ui.axTilt); cla(ui.axAlt); cla(ui.axXY); cla(ui.axSon);

        plot(ui.axTilt, res.t, res.tilt, 'LineWidth',1.4, 'Color',[0.45 0.10 0.55]);
        yline(ui.axTilt, 45, '--', 'sonar gate 45 deg');
        ylim(ui.axTilt, [0 max(50, max(res.tilt)*1.1)]);

        plot(ui.axAlt, res.t, res.Z, 'LineWidth',1.5, 'Color',[0.10 0.10 0.10]); hold(ui.axAlt,'on');
        yline(ui.axAlt, cfg.hoverZ, '--', 'target altitude');
        yline(ui.axAlt, 0, '-', 'ground', 'Color',[0.7 0.2 0.1]);
        hold(ui.axAlt,'off');

        % Only the evaluated stretch is drawn. A crashed airframe keeps sliding
        % along the ground afterwards, and that tail alone sets the axis scale.
        plot(ui.axXY, res.X(res.w), res.Y(res.w), 'LineWidth',1.3, 'Color',[0.10 0.35 0.70]);
        hold(ui.axXY,'on');
        plot(ui.axXY, res.target(1), res.target(2), 'p', 'MarkerSize',13, ...
            'MarkerFaceColor',[0.95 0.75 0.10], 'MarkerEdgeColor','k');
        plot(ui.axXY, res.X(res.iEnd), res.Y(res.iEnd), 'o', 'MarkerSize',8, ...
            'MarkerFaceColor',[0.85 0.25 0.15], 'MarkerEdgeColor','k');
        legend(ui.axXY, {'track','target','final'}, 'Location','best', 'FontSize',8);
        axis(ui.axXY,'equal'); hold(ui.axXY,'off');

        area(ui.axSon, res.t, res.gate, 'FaceColor',[0.72 0.85 0.98], 'EdgeColor','none');
        ylim(ui.axSon, [-0.1 1.1]);
        title(ui.axSon, sprintf('Sonar valid window (%s)', res.gateSrc));

        if res.crashed
            for ax = [ui.axTilt ui.axAlt ui.axSon]
                xline(ax, res.tHit, ':', 'ground contact', 'Color',[0.80 0.15 0.10], 'LineWidth',1.2);
            end
        end

        lines = {};
        if res.preCrashed
            lines{end+1} = sprintf(['[!] The airframe was already on the ground (%.2f s) before the\n' ...
                '    scenario even started. The baseline flight failed on its own, so every\n' ...
                '    metric below is meaningless. Check the controller/estimator first.'], res.tHit);
            lines{end+1} = '';
        end
        if strcmp(cfg.mode,'impact')
            lines{end+1} = sprintf('Impact    : J = %.3f N.s,  contact %.0f ms,  t = %.1f s', ...
                cfg.J, cfg.dur*1000, cfg.t0);
            lines{end+1} = sprintf('Hit point : r = [%.1f, %.1f, %.1f] cm (body frame)', cfg.r*100);
        else
            lines{end+1} = sprintf('Initial   : tilt %.0f deg,  release %.1f m,  motor hold %.2f s', ...
                cfg.tilt0, cfg.alt0, cfg.hold);
        end
        lines{end+1} = sprintf('Peak tilt      : %.1f deg', res.maxTilt);
        if isnan(res.settle)
            lines{end+1} = 'Recovery       : never got below 5 deg within the run';
        else
            lines{end+1} = sprintf('Recovery       : %.2f s (to below 5 deg)', res.settle);
        end
        if res.crashed
            lines{end+1} = sprintf('Min altitude   : %.2f m   <- ground contact at %.2f s', res.minAlt, res.tHit);
        else
            lines{end+1} = sprintf('Min altitude   : %.2f m', res.minAlt);
        end
        lines{end+1} = sprintf('Max excursion  : %.3f m horizontal (from target)', res.maxDev);
        lines{end+1} = sprintf('Final error    : %.3f m horizontal,  %+.3f m altitude,  tilt %.1f deg', ...
            res.errXY, res.errZ, res.endTilt);
        lines{end+1} = sprintf('Sonar duty     : %.0f%% (%s)', res.gateFrac*100, res.gateSrc);
        lines{end+1} = sprintf('Eval window    : %.2f - %.2f s%s', cfg.evalFrom, res.tEval, ...
            tern(res.crashed, ' (after contact excluded: it slides along the ground)', ''));
        ui.scoreTxt.Text = strjoin(lines, newline);

        if res.success
            ui.verdict.Text = 'SUCCESS';
            ui.verdict.FontColor = [0.05 0.50 0.18];
        else
            ui.verdict.Text = 'FAILURE';
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
