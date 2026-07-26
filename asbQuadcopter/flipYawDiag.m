function T = flipYawDiag(Avec, opts)
% flipYawDiag  대각선 외란에서 "요가 진짜 발산 원인인가" 진단.
%   대각선 [A A 0] 펄스로 여러 A 실행 → 요각/요속도/기울기/수평이탈을 겹쳐 그림.
%   목적: 발산(큰 이탈)이 요각/요속도 발산과 동기화되는지 실측 확인.
%
%   Avec : 대각선 진폭 벡터 (예: [0.16 0.19 0.22 0.30])
%   opts : .stop(기본22) .t0(기본8) .width(기본0.02)
%   반환 T : A별 yaw 특화 지표 테이블. 겹침그래프 flipResults/yawdiag/ 에 저장.

    arguments
        Avec (1,:) double
        opts.stop (1,1) double = 22
        opts.t0   (1,1) double = 8
        opts.width(1,1) double = 0.02
    end
    mdl='asbQuadcopter';
    stepA='nonlinearAirframe/Nonlinear/Step'; stepB='nonlinearAirframe/Nonlinear/Step1';
    orig=struct('A',get_param(stepA,'After'),'B',get_param(stepB,'After'), ...
        'TA',get_param(stepA,'Time'),'TB',get_param(stepB,'Time'),'stop',get_param(mdl,'StopTime'), ...
        'sf',get_param(mdl,'StopFcn'));
    cu=onCleanup(@() i_restore(stepA,stepB,mdl,orig));
    set_param(mdl,'StopFcn','');
    set_param(stepA,'Time',num2str(opts.t0)); set_param(stepB,'Time',num2str(opts.t0+opts.width));

    outdir=fullfile(pwd,'flipResults','yawdiag'); if ~exist(outdir,'dir'); mkdir(outdir); end
    n=numel(Avec);
    [maxTilt,maxYaw,maxYawRate,finalYaw,maxDev,finalDev]=deal(nan(n,1));
    C=lines(n); traces=cell(n,1);

    for i=1:n
        A=Avec(i); vec=sprintf('[%g %g 0]',A,A);
        set_param(stepA,'After',vec); set_param(stepB,'After',vec);
        fprintf('  [%d/%d] diag A=%.3f...\n',i,n,A);
        in=Simulink.SimulationInput(mdl); in=in.setModelParameter('StopTime',num2str(opts.stop));
        out=sim(in); ls=out.log_states;
        t=ls.Euler.Time; D=ls.DCM_be.Data;
        R33=squeeze(D(3,3,:)); R21=squeeze(D(2,1,:)); R11=squeeze(D(1,1,:));
        tilt=acosd(min(max(R33,-1),1));
        yaw=unwrap(atan2(R21,R11))*180/pi; yaw=yaw-yaw(1);   % 요각(연속,deg)
        Om=squeeze(ls.Omega_body.Data); if size(Om,1)==3,Om=Om'; end
        r=Om(:,3);                                           % 요속도 rad/s
        X=squeeze(ls.X_ned.Data); if size(X,1)==3&&size(X,2)~=3,X=X'; end
        dev=sqrt(sum((X(:,1:2)-X(1,1:2)).^2,2));

        traces{i}=struct('t',t,'tilt',tilt,'yaw',yaw,'r',r,'dev',dev,'A',A);
        maxTilt(i)=max(tilt); maxYaw(i)=max(abs(yaw)); maxYawRate(i)=max(abs(r));
        finalYaw(i)=yaw(end); maxDev(i)=max(dev); finalDev(i)=dev(end);
    end

    T=table(Avec(:),maxTilt,maxYaw,maxYawRate,finalYaw,maxDev,finalDev, ...
        'VariableNames',{'A','maxTilt','maxYaw_deg','maxYawRate','finalYaw_deg','maxDev_m','finalDev_m'});
    disp(' '); disp('===== 대각선 요 진단 ====='); disp(T);

    % 겹침 그래프
    f=figure('Visible','off','Color','k','Position',[60 60 1100 820]);
    lab=arrayfun(@(a)sprintf('A=%.2f',a),Avec,'uni',0);
    ax1=subplot(2,2,1); ax2=subplot(2,2,2); ax3=subplot(2,2,3); ax4=subplot(2,2,4);
    for i=1:n
        s=traces{i};
        plot(ax1,s.t,s.tilt,'Color',C(i,:),'LineWidth',1.4); hold(ax1,'on');
        plot(ax2,s.t,s.yaw,'Color',C(i,:),'LineWidth',1.4); hold(ax2,'on');
        plot(ax3,s.t,s.r,'Color',C(i,:),'LineWidth',1.4); hold(ax3,'on');
        plot(ax4,s.t,s.dev,'Color',C(i,:),'LineWidth',1.4); hold(ax4,'on');
    end
    title(ax1,'기울기 [deg]'); yline(ax1,90,'--','','Color',[1 .5 0]);
    title(ax2,'요각 [deg] (누적)');
    title(ax3,'요속도 r [rad/s]'); xlabel(ax3,'t [s]');
    title(ax4,'수평이탈 [m]'); xlabel(ax4,'t [s]');
    legend(ax4,lab,'TextColor','w','Color','k');
    for ax=[ax1 ax2 ax3 ax4]; set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.5 .5 .5]); grid(ax,'on'); end
    set(findall(f,'Type','text'),'Color','w');
    sgtitle(f,'대각선 외란: 발산 vs 요 발산 동기화 확인','Color','w');
    exportgraphics(f,fullfile(outdir,'yaw_diag_overlay.png'),'BackgroundColor','k'); close(f);
    save(fullfile(outdir,'yawdiag.mat'),'T'); writetable(T,fullfile(outdir,'yawdiag.csv'));
    fprintf('저장 → %s\n',outdir);
end

function i_restore(stepA,stepB,mdl,o)
    set_param(stepA,'After',o.A); set_param(stepB,'After',o.B);
    set_param(stepA,'Time',o.TA); set_param(stepB,'Time',o.TB);
    set_param(mdl,'StopTime',o.stop); set_param(mdl,'StopFcn',o.sf);
end
