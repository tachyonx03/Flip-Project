function ttPlot4(sets, outDir, fname)
%TTPLOT4  제어기 여러 안을 한 장에 비교 (검은 배경, 영문 라벨).
%
%   ttPlot4(sets, outDir)          기본 파일명 tt_compare4.png
%   ttPlot4(sets, outDir, fname)   파일명 지정
%
%   sets = struct array, 각 원소 필드:
%     name  범례 문자열 (TeX 가능, 예 'w_{yaw} binary')
%     A     외란 진폭 벡터
%     S     ttRun 결과 구조체 배열 (A 와 길이 일치)
%     col   [r g b]
%     sty   plot 스타일 (예 'o-','s--','^-.','d-')
%
%   A 는 자동 정렬·중복 제거되므로 거친 스윕과 정밀 스윕을 이어 붙여도 됨.
%
%   4분할 출력: 최대이탈(log) / 요 감김 / 텀블 횟수 / 최종 잔여오차(log)
%   콘솔에 최대이탈 표 + 최악값도 같이 찍음.
%
%   예)
%     sets = struct('name',{'Baseline','Hybrid'}, ...
%                   'A',{Avec,Avec}, 'S',{BASE,HY}, ...
%                   'col',{[1 1 1],[.4 1 .4]}, 'sty',{'o-','d-'});
%     ttPlot4(sets, 'flipResults/tiltTorsion');
%
%   See also TTRUN.

if nargin < 3, fname = 'tt_compare4.png'; end
if ~exist(outDir,'dir'), mkdir(outDir); end

% A 오름차순 정렬 + 중복 제거 (거친/정밀 스윕 병합 대비)
for kk = 1:numel(sets)
    [au, ia] = unique(sets(kk).A, 'stable');
    [au, io] = sort(au);
    ia = ia(io);
    sets(kk).A = au;
    sets(kk).S = sets(kk).S(ia);
end

fh   = figure('Color','k','Position',[60 30 1100 900],'Name','ttPlot4');
axh  = gobjects(1,4);
mets = {'maxDev','maxYaw','nCross90','finalDev'};
ylab = {'Max deviation [m]','Peak |yaw| [deg]', ...
        '# of 90^\circ crossings','Final deviation [m]'};
tit  = {'Peak horizontal excursion','Yaw winding', ...
        'Tumble count','Residual error at StopTime'};
logy = [true false false true];

for pp = 1:4
    axh(pp) = subplot(2,2,pp);
    for kk = 1:numel(sets)
        yv = arrayfun(@(x) x.(mets{pp}), sets(kk).S);
        if logy(pp), yv = max(yv, 1e-3); end      % log 축 하한
        plot(sets(kk).A, yv, sets(kk).sty, 'Color', sets(kk).col, ...
             'LineWidth', 1.7, 'MarkerFaceColor', sets(kk).col, 'MarkerSize', 6);
        hold on;
    end
    if logy(pp), set(gca,'YScale','log'); end
    xlabel('Disturbance A'); ylabel(ylab{pp}); title(tit{pp});
    if pp == 1, legend({sets.name}, 'Location','northwest'); end
end

% 검은 배경 스타일 일괄 적용
set(axh,'Color','k','XColor','w','YColor','w', ...
        'GridColor',[.8 .8 .8],'GridAlpha',.3);
arrayfun(@(a) grid(a,'on'), axh);
set(findall(fh,'Type','text'),'Color','w');
set(findobj(fh,'Type','legend'),'TextColor','w','Color','k','EdgeColor','w');
exportgraphics(fh, fullfile(outDir,fname),'BackgroundColor','k','Resolution',150);
fprintf('saved: %s\n', fullfile(outDir,fname));

% 최대이탈 표
fprintf('\n%-7s', 'A');
fprintf('%14s', sets.name); fprintf('\n');
for aa = unique([sets.A])
    fprintf('%-7.2f', aa);
    for kk = 1:numel(sets)
        j = find(abs(sets(kk).A - aa) < 1e-9, 1);
        if isempty(j), fprintf('%14s','-');
        else,          fprintf('%14.2f', sets(kk).S(j).maxDev); end
    end
    fprintf('\n');
end
fprintf('%-7s','WORST');
for kk = 1:numel(sets)
    fprintf('%14.2f', max(arrayfun(@(x) x.maxDev, sets(kk).S)));
end
fprintf('\n');
end
