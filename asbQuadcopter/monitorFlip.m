%% monitorFlip.m — 6DOF 드론 회전상태 모니터
%  현재 모델 그대로 시뮬 돌리고, 드론이 얼마나 도는지 그림으로 보여줌.
%  외란 세기 바꿀 때마다 이거 실행해서 확인하면 됨.
clc;
in = Simulink.SimulationInput('asbQuadcopter');   % 결과를 항상 객체로 받게
so = sim(in);

ls  = so.log_states;
t   = ls.Euler.Time;
D   = ls.DCM_be.Data;               % 3x3xN 회전행렬
R33 = squeeze(D(3,3,:));
tilt = acosd(R33);                  % 기울기: 0=수평, 90=옆, 180=완전 뒤집힘 (짐벌락 없음)
Om  = squeeze(ls.Omega_body.Data); if size(Om,1)==3, Om = Om'; end
q   = Om(:,2);                      % pitch 회전속도 [rad/s]

fprintf('최대 기울기 : %.1f deg  (90° 넘어야 freeze 진입, 180° 넘겨야 플립완주)\n', max(tilt));
fprintf('최대 회전속도: %.1f rad/s\n', max(abs(q)));

figure('Name','flip monitor','Color','k','Position',[100 100 820 620]);

ax1 = subplot(2,1,1);
plot(t, tilt, 'c', 'LineWidth', 1.6); hold on;
yline(90 ,'--','90° (freeze 진입)','Color',[1 .5 0],'LineWidth',1.3,'LabelHorizontalAlignment','left');
yline(180,':' ,'180° (뒤집힘 꼭대기)','Color',[1 1 1],'LabelHorizontalAlignment','left');
ylabel('기울기 [deg]'); title('드론 기울기 (0=수평, 180=완전 뒤집힘)');

ax2 = subplot(2,1,2);
plot(t, q, 'g', 'LineWidth', 1.4);
ylabel('pitch 회전속도 [rad/s]'); xlabel('시간 [s]'); title('회전 속도');

for ax = [ax1 ax2]
    set(ax,'Color','k','XColor','w','YColor','w','GridColor',[.5 .5 .5]); grid(ax,'on');
end
set(findall(gcf,'Type','text'),'Color','w');
