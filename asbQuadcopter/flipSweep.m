function T = flipSweep(mode, Avec, opts)
% flipSweep  외란 세기 스윕 → 플립 지표 수집 + 그래프/데이터 저장.
%   mode : 'single' (순수피치 [0 A 0])  또는  'diag' (대각선 [A A 0])
%   Avec : 외란 진폭 벡터 (예: [0.12 0.16 0.20 0.24 0.28])
%   opts : (선택) struct — .stop(StopTime,기본22) .t0(외란시각,기본8) .width(펄스폭,기본0.02)
%
%   반환 T : 결과 테이블. 그래프 png + T(.mat/.csv)를 flipResults/<mode>/ 에 저장.
%   지표: maxTilt(최대기울기) netRot(순회전각) nFlips(바퀴수) finalTilt(최종기울기)
%         finalRate(최종회전속도) maxDev(최대XY이탈) settle(정착시간) verdict(판정)

    arguments
        mode  (1,:) char
        Avec  (1,:) double
        opts.stop  (1,1) double = 22
        opts.t0    (1,1) double = 8
        opts.width (1,1) double = 0.02
    end

    mdl = 'asbQuadcopter';
    stepA = 'nonlinearAirframe/Nonlinear/Step';
    stepB = 'nonlinearAirframe/Nonlinear/Step1';

    % --- 원상복구용 현재값 저장 ---
    orig.After  = get_param(stepA,'After');
    orig.AfterB = get_param(stepB,'After');
    orig.TimeA  = get_param(stepA,'Time');
    orig.TimeB  = get_param(stepB,'Time');
    orig.stop   = get_param(mdl,'StopTime');
    cleanupObj = onCleanup(@() restore(stepA,stepB,mdl,orig));

    % --- 출력 폴더 ---
    outdir = fullfile(pwd,'flipResults',mode);
    if ~exist(outdir,'dir'); mkdir(outdir); end

    % --- 펄스 시각/폭 세팅 (진폭만 루프에서 바꿈) ---
    set_param(stepA,'Time',num2str(opts.t0));
    set_param(stepB,'Time',num2str(opts.t0+opts.width));
    set_param(mdl,'StopTime',num2str(opts.stop));

    n = numel(Avec);
    [maxTilt,netRot,nFlips,finalTilt,finalRate,maxDev,settle] = deal(nan(n,1));
    verdict = strings(n,1);

    for i = 1:n
        A = Avec(i);
        if strcmp(mode,'single'); vec = sprintf('[0 %g 0]', A);
        else;                     vec = sprintf('[%g %g 0]', A, A); end
        set_param(stepA,'After',vec);
        set_param(stepB,'After',vec);

        fprintf('  [%d/%d] %s A=%.3f 실행...\n', i, n, mode, A);
        in = Simulink.SimulationInput(mdl);
        in = in.setModelParameter('StopTime',num2str(opts.stop));
        out = sim(in);                    % 함수 컨텍스트서 안전하게 out에서 읽음
        ls = out.log_states;

        m = i_metrics(ls, opts.t0);
        maxTilt(i)=m.maxTilt; netRot(i)=m.netRot; nFlips(i)=m.nFlips;
        finalTilt(i)=m.finalTilt; finalRate(i)=m.finalRate;
        maxDev(i)=m.maxDev; settle(i)=m.settle; verdict(i)=m.verdict;

        % 개별 시간이력 저장
        i_saveTrace(m, outdir, mode, A);
    end

    T = table(Avec(:), maxTilt, netRot, nFlips, finalTilt, finalRate, maxDev, settle, verdict, ...
        'VariableNames', {'A','maxTilt','netRot','nFlips','finalTilt','finalRate','maxDev','settle','verdict'});
    disp(' '); disp(['===== ' upper(mode) ' 스윕 결과 =====']); disp(T);

    % 요약 그래프 + 데이터 저장
    i_summary(T, mode, outdir);
    save(fullfile(outdir,['sweep_' mode '.mat']),'T');
    writetable(T, fullfile(outdir,['sweep_' mode '.csv']));
    fprintf('저장 완료 → %s\n', outdir);
end

% ================= 지표 추출 =================
function m = i_metrics(ls, t0)
    t = ls.Euler.Time;
    D = ls.DCM_be.Data;                       % 3x3xN
    R13 = squeeze(D(1,3,:)); R33 = squeeze(D(3,3,:));
    tilt = acosd(min(max(R33,-1),1));         % 0~180 (짐벌락 없음)
    th = unwrap(atan2(R13,R33))*180/pi; th = th - th(1);
    Om = squeeze(ls.Omega_body.Data); if size(Om,1)==3, Om=Om'; end
    ratemag = sqrt(sum(Om.^2,2));

    X = squeeze(ls.X_ned.Data); if size(X,1)==3 && size(X,2)~=3, X=X'; end
    xy = X(:,1:2); xy0 = xy(1,:);
    dev = sqrt(sum((xy-xy0).^2,2));

    post = t >= t0;                           % 외란 이후 구간
    m.maxTilt   = max(tilt);
    m.netRot    = th(end);
    m.nFlips    = round(abs(th(end))/360);
    m.finalTilt = tilt(end);
    m.finalRate = ratemag(end);
    m.maxDev    = max(dev(post));

    % 정착시간: 이후로 계속 tilt<5° & rate<0.5 인 마지막 진입점 (외란시각 기준)
    unsettled = (tilt>5) | (ratemag>0.5);
    idx = find(unsettled & post, 1, 'last');
    if isempty(idx); m.settle = 0; else; m.settle = t(min(idx+1,numel(t))) - t0; end

    % 판정
    if m.maxTilt < 90
        m.verdict = "no-flip";
    elseif abs(m.netRot) < 150
        m.verdict = "return";                 % 90° 넘었다 되돌림
    elseif m.finalTilt < 10 && m.finalRate < 0.5
        m.verdict = sprintf("flip x%d OK", m.nFlips);
    else
        m.verdict = sprintf("flip x%d UNSETTLED", m.nFlips);
    end

    % 플롯용 원자료
    m.t=t; m.tilt=tilt; m.th=th; m.rate=ratemag; m.xy=xy; m.xy0=xy0;
end

% ============== 개별 시간이력 png ==============
function i_saveTrace(m, outdir, mode, A)
    f = figure('Visible','off','Color','k','Position',[80 80 900 720]);
    ax1=subplot(2,2,1); plot(ax1,m.t,m.th,'c','LineWidth',1.5); yline(ax1,360,'--','','Color',[1 .5 0]); yline(ax1,-360,'--','','Color',[1 .5 0]);
    title(ax1,'회전각 [deg]');
    ax2=subplot(2,2,2); plot(ax2,m.t,m.tilt,'c','LineWidth',1.5); yline(ax2,90,'--','','Color',[1 .5 0]); yline(ax2,180,':','','Color','w');
    title(ax2,'기울기 [deg]');
    ax3=subplot(2,2,3); plot(ax3,m.t,m.rate,'g','LineWidth',1.4); xlabel(ax3,'t [s]'); title(ax3,'회전속도 [rad/s]');
    ax4=subplot(2,2,4); plot(ax4,m.xy(:,1)-m.xy0(1), m.xy(:,2)-m.xy0(2),'m','LineWidth',1.3); hold(ax4,'on');
    plot(ax4,0,0,'wo'); axis(ax4,'equal'); xlabel(ax4,'ΔX [m]'); ylabel(ax4,'ΔY [m]'); title(ax4,'수평 이동궤적');
    for ax=[ax1 ax2 ax3 ax4]; set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.5 .5 .5]); grid(ax,'on'); end
    set(findall(f,'Type','text'),'Color','w');
    sgtitle(f, sprintf('%s A=%.3f  |  %s', mode, A, m.verdict), 'Color','w');
    exportgraphics(f, fullfile(outdir, sprintf('trace_%s_A%03d.png', mode, round(A*1000))), 'BackgroundColor','k');
    close(f);
end

% ============== 요약 그래프 ==============
function i_summary(T, mode, outdir)
    f = figure('Visible','off','Color','k','Position',[80 80 1000 720]);
    A = T.A;
    ax1=subplot(2,2,1); plot(ax1,A,T.maxTilt,'-oc','LineWidth',1.6); yline(ax1,180,':','완주선','Color','w'); yline(ax1,90,'--','','Color',[1 .5 0]);
    ylabel(ax1,'최대기울기 [deg]'); title(ax1,'최대 기울기');
    ax2=subplot(2,2,2); plot(ax2,A,T.nFlips,'-s','Color',[1 .5 0],'LineWidth',1.6); ylabel(ax2,'바퀴수'); title(ax2,'완주 바퀴수'); ylim(ax2,[-.2 max(3,max(T.nFlips))+.5]);
    ax3=subplot(2,2,3); plot(ax3,A,T.maxDev,'-om','LineWidth',1.6); xlabel(ax3,'외란 진폭 A'); ylabel(ax3,'최대 XY이탈 [m]'); title(ax3,'수평 이탈');
    ax4=subplot(2,2,4); plot(ax4,A,T.settle,'-og','LineWidth',1.6); xlabel(ax4,'외란 진폭 A'); ylabel(ax4,'정착시간 [s]'); title(ax4,'정착시간(외란기준)');
    for ax=[ax1 ax2 ax3 ax4]; set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.5 .5 .5]); grid(ax,'on'); end
    set(findall(f,'Type','text'),'Color','w');
    sgtitle(f, sprintf('%s 외란 스윕 요약 (Ts=%.3f)', mode, evalin('base','Ts')), 'Color','w');
    exportgraphics(f, fullfile(outdir, sprintf('summary_%s.png', mode)), 'BackgroundColor','k');
    close(f);
end

% ============== 원상복구 ==============
function restore(stepA,stepB,mdl,orig)
    set_param(stepA,'After',orig.After);  set_param(stepB,'After',orig.AfterB);
    set_param(stepA,'Time',orig.TimeA);   set_param(stepB,'Time',orig.TimeB);
    set_param(mdl,'StopTime',orig.stop);
end
