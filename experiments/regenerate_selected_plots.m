function regenerate_selected_plots()
thisDir = fileparts(mfilename('fullpath'));
repositoryRoot = fileparts(thisDir);
dataDir = fullfile(repositoryRoot, 'data');
outputDir = fullfile(repositoryRoot, 'results');
plotDir = fullfile(outputDir, 'figures');
loaded = load(fullfile(outputDir, 'ekf_case_results.mat'), 'results');
results = loaded.results;

sens = load(fullfile(dataDir, 'Sens_data.mat'), 'gps');

make_gnss_anomaly(result_by_id(results,'BASE'), sens.gps, ...
    fullfile(plotDir,'gnss_quality_anomaly.png'), ...
    fullfile(outputDir,'gnss_anomaly_metrics.csv'));
make_mitigation_comparison(results, fullfile(plotDir,'mitigation_rmse_comparison.png'));
end


function r = result_by_id(results,id)
r = results(find(strcmp({results.id},id),1,'first'));
end


function finish_plot(fig,outPath)
set(findall(fig,'-property','FontName'),'FontName','Arial');
exportgraphics(fig,outPath,'Resolution',220,'BackgroundColor','white');
close(fig);
end


function make_gnss_anomaly(base,gps,outPath,metricsPath)
[~,uniqueIdx] = unique(double(gps.gpsTow(:)),'stable');
gpsTimeAbsolute = double(gps.timeGpsTow(uniqueIdx));
alignEndAbsolute = double(gps.timeGpsTow(1)) + 300;
gpsTime = gpsTimeAbsolute - alignEndAbsolute;
gpsEcef = double(gps.gpsPosEcef(uniqueIdx,:));
gpsNed = (base.Ce2n * (gpsEcef - base.originEcef).').';
ref = nan(size(gpsNed));
for axisIdx=1:3
    ref(:,axisIdx)=interp1(base.time,base.refPos(:,axisIdx),gpsTime,'linear',nan);
end
err = gpsNed-ref;
horizontalError=hypot(err(:,1),err(:,2));
posU=double(gps.gpsPosU(uniqueIdx,:));
sat=double(gps.gpsNumSats(uniqueIdx));
window=gpsTime>=130 & gpsTime<=230;

[maxHorizontal,localIdx]=max(horizontalError(window));
windowIdx=find(window);
peakIdx=windowIdx(localIdx);
metrics=table(maxHorizontal,gpsTime(peakIdx),err(peakIdx,1),err(peakIdx,2),err(peakIdx,3), ...
    sat(peakIdx),posU(peakIdx,1),posU(peakIdx,2),posU(peakIdx,3), ...
    'VariableNames',{'MaxHorizontalError_m','PeakTimeAfterAlignment_s','NorthError_m','EastError_m','DownError_m', ...
    'Satellites','PosU_N_sigma_m','PosU_E_sigma_m','PosU_D_sigma_m'});
writetable(metrics,metricsPath);

fig=figure('Visible','off','Color','w','Position',[50 50 1180 760]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(gpsTime(window),err(window,1),'LineWidth',1.2); hold on;
plot(gpsTime(window),err(window,2),'LineWidth',1.2);
plot(gpsTime(window),horizontalError(window),'Color',[0.976 0.451 0.086],'LineWidth',1.7);
grid on; yline(0,':'); ylabel('Position error [m]');
title(sprintf('절대시각 약 %.0f s (정렬 후 %.0f s): Raw GNSS 순간 이탈', ...
    gpsTimeAbsolute(peakIdx)-gpsTimeAbsolute(1),gpsTime(peakIdx)));
legend('North','East','Horizontal norm','Location','best');
nexttile;
plot(gpsTime(window),posU(window,:),'LineWidth',1.15); grid on; ylabel('gpsPosU \sigma [m]');
legend('N','E','D','Location','best');
nexttile;
stairs(gpsTime(window),sat(window),'Color',[0.086 0.271 0.604],'LineWidth',1.35);
grid on; ylabel('Satellites'); xlabel('Time after alignment [s]');
finish_plot(fig,outPath);
end


function make_mitigation_comparison(results,outPath)
ids={'BASE','R_adaptive_no_clamp','R_adaptive_K15_zero','R_fixed_full_Kgyro_zero'};
names={'고정 R / 제한 없음','Adaptive R / 제한 없음','Adaptive R / K(15,:)=0','고정 R / K(13:15,:)=0'};
pos=zeros(4,3); yaw=zeros(4,1);
for k=1:4
    r=result_by_id(results,ids{k}); pos(k,:)=r.posRmse; yaw(k)=r.attRmse(3);
end
colors=[0.55 0.55 0.55;0.176 0.647 0.839;0.976 0.451 0.086;0.086 0.271 0.604];
fig=figure('Visible','off','Color','w','Position',[50 50 1220 720]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; b=bar(pos.'); grid on; ylabel('Position RMSE [m]'); title('위치: Adaptive R의 Down 개선');
set(gca,'XTickLabel',{'North','East','Down'});
for k=1:numel(b), b(k).FaceColor=colors(k,:); end
legend(names,'Location','northoutside','NumColumns',2);
nexttile; b=bar(yaw,'FaceColor','flat'); grid on; ylabel('Yaw RMSE [deg]'); title('Yaw: Z축 자이로 바이어스 제한의 영향');
set(gca,'XTick',1:4,'XTickLabel',{'고정R','AdaptiveR','AdaptiveR+K15','고정R+K13:15'},'XTickLabelRotation',18);
for k=1:4
    b.CData(k,:)=colors(k,:);
    text(k,yaw(k)+1.0,sprintf('%.2f°',yaw(k)),'HorizontalAlignment','center');
end
finish_plot(fig,outPath);
end
