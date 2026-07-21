function flipGraph()
% flipGraph  시뮬 끝나면 자동 호출되는 회전상태 그래프 (asbQuadcopter StopFcn 콜백)
%   회전각(0→360=한바퀴) / 기울기(0→180) / 회전속도 를 검은 배경에 그림.
%   기울기는 회전행렬(DCM)에서 뽑아 짐벌락 없음.

    if ~evalin('base','exist(''log_states'',''var'')')
        return;   % 로그 아직 없으면 조용히 패스
    end
    ls = evalin('base','log_states');

    t   = ls.Euler.Time;
    D   = ls.DCM_be.Data;                         % 3x3xN
    R13 = squeeze(D(1,3,:));  R33 = squeeze(D(3,3,:));
    tilt = acosd(min(max(R33,-1),1));             % 기울기 0~180 (짐벌락 없음)
    th   = unwrap(atan2(R13,R33))*180/pi;         % 실제 회전각(연속)
    th   = th - th(1);
    Om = squeeze(ls.Omega_body.Data); if size(Om,1)==3, Om=Om'; end
    q  = Om(:,2);

    % 같은 창 재사용 (매 실행마다 새 창 안 뜨게)
    f = findobj('Type','figure','Tag','flipMon');
    if isempty(f), f = figure('Tag','flipMon','Name','flip monitor','Color','k','Position',[80 80 850 700]);
    else, clf(f); set(f,'Color','k'); figure(f); end

    ax1 = subplot(3,1,1,'Parent',f);
    plot(ax1,t,th,'c','LineWidth',1.6); hold(ax1,'on');
    yline(ax1,360,'--','한 바퀴(360°)','Color',[1 .5 0],'LineWidth',1.3);
    ylabel(ax1,'회전각 [deg]'); title(ax1,'회전각 (0→±360 = 한 바퀴 완주)');

    ax2 = subplot(3,1,2,'Parent',f);
    plot(ax2,t,tilt,'c','LineWidth',1.6); hold(ax2,'on');
    yline(ax2,90,'--','90° freeze','Color',[1 .5 0]); yline(ax2,180,':','뒤집힘','Color','w');
    ylabel(ax2,'기울기 [deg]'); title(ax2,'기울기 (0=수평, 180=완전 뒤집힘)');

    ax3 = subplot(3,1,3,'Parent',f);
    plot(ax3,t,q,'g','LineWidth',1.4);
    ylabel(ax3,'회전속도 [rad/s]'); xlabel(ax3,'시간 [s]'); title(ax3,'회전속도 (0으로 잦아들면 정착)');

    for ax=[ax1 ax2 ax3]
        set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.5 .5 .5]); grid(ax,'on');
    end
    set(findall(f,'Type','text'),'Color','w');
    fprintf('[flipGraph] 최대기울기 %.0f°, 순회전각 %.0f°, 최종회전속도 %.1f rad/s\n', max(tilt), th(end), q(end));
end
