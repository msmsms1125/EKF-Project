function run_all_experiments()
% Reproduce the P/Q tuning sequence and the later R/K mitigation tests.

close all force;
clc;

thisDir = fileparts(mfilename('fullpath'));
repositoryRoot = fileparts(thisDir);
sourceDir = fullfile(repositoryRoot, 'src');
dataDir = fullfile(repositoryRoot, 'data');
outputDir = fullfile(repositoryRoot, 'results');
plotDir = fullfile(outputDir, 'figures');
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
if ~exist(plotDir, 'dir'), mkdir(plotDir); end

addpath(fullfile(sourceDir, 'geometric'));
addpath(fullfile(sourceDir, 'navigation'));
addpath(fullfile(sourceDir, 'INSToolbox'));

sens = load(fullfile(dataDir, 'Sens_data.mat'));
truth = load(fullfile(dataDir, 'True_data.mat'));
imu = sens.imu;
gps = sens.gps;
Ins = truth.Ins;

cases = build_cases();
results = repmat(empty_result(), numel(cases), 1);

fprintf('Running %d EKF cases...\n', numel(cases));
for k = 1:numel(cases)
    fprintf('\n[%02d/%02d] %s\n', k, numel(cases), cases(k).label);
    results(k) = run_one_case(imu, gps, Ins, cases(k));
    fprintf('  RMSE pos = [%.4f %.4f %.4f] m\n', results(k).posRmse);
    fprintf('  RMSE vel = [%.4f %.4f %.4f] m/s\n', results(k).velRmse);
    fprintf('  RMSE att = [%.4f %.4f %.4f] deg\n', results(k).attRmse);
end

metrics = results_to_table(results);
writetable(metrics, fullfile(outputDir, 'ekf_case_metrics.csv'));
save(fullfile(outputDir, 'ekf_case_results.mat'), 'results', 'cases', '-v7.3');

make_all_plots(results, gps, plotDir);
fprintf('\nSaved metrics and plots to:\n%s\n', outputDir);
end


function cases = build_cases()
ids = { ...
    'BASE', ...
    'P1_all_x10', 'P2_all_x0p1', ...
    'P3_gyro_bias_x0p1', 'P4_gyro_bias_x0p01', ...
    'P5_gyro_z_bias_x0p1', 'P6_gyro_z_bias_x0p01', ...
    'P7_yaw_x0p1', 'P8_yaw_x10', ...
    'Q1_all_x10', 'Q2_all_x0p1', ...
    'Q3_acc_all_x10', 'Q4_acc_all_x0p1', ...
    'Q5_gyro_all_x10', 'Q6_gyro_all_x0p1', ...
    'Q7_acc_z_x10', 'Q8_gyro_z_x10', ...
    'R_fixed_full_Kgyro_zero', ...
    'R_adaptive_no_clamp', ...
    'R_adaptive_K15_zero'};

labels = { ...
    '기본 P/Q · 고정 R · K 제한 없음', ...
    'P Case 1: P 전체 ×10', 'P Case 2: P 전체 ×0.1', ...
    'P Case 3: 자이로 바이어스 P ×0.1', 'P Case 4: 자이로 바이어스 P ×0.01', ...
    'P Case 5: Z 자이로 바이어스 P ×0.1', 'P Case 6: Z 자이로 바이어스 P ×0.01', ...
    'P Case 7: Yaw P ×0.1', 'P Case 8: Yaw P ×10', ...
    'Q Case 1: Q 전체 ×10', 'Q Case 2: Q 전체 ×0.1', ...
    'Q Case 3: 가속도 Q 전체 ×10', 'Q Case 4: 가속도 Q 전체 ×0.1', ...
    'Q Case 5: 자이로 Q 전체 ×10', 'Q Case 6: 자이로 Q 전체 ×0.1', ...
    'Q Case 7: Z 가속도 Q ×10', 'Q Case 8: Z 자이로 Q ×10', ...
    '고정 R + K(13:15,:)=0', ...
    'Adaptive position R + K 제한 없음', ...
    'Adaptive position R + K(15,:)=0'};

cases = repmat(struct('id', '', 'label', '', 'tuning', '', 'mode', ''), numel(ids), 1);
for k = 1:numel(ids)
    cases(k).id = ids{k};
    cases(k).label = labels{k};
    cases(k).tuning = ids{k};
    cases(k).mode = 'fixed_no_clamp';
end
cases(end-2).mode = 'fixed_full_clamp';
cases(end-1).mode = 'adaptive_no_clamp';
cases(end).mode = 'adaptive_k15';
end


function out = empty_result()
out = struct( ...
    'id', '', 'label', '', 'mode', '', ...
    'posRmse', nan(1,3), 'velRmse', nan(1,3), 'attRmse', nan(1,3), ...
    'posMax', nan(1,3), 'velMax', nan(1,3), 'attMax', nan(1,3), ...
    'finalGyroBias', nan(1,3), 'finalAccelBias', nan(1,3), ...
    'time', [], 'posError', [], 'velError', [], 'attError', [], ...
    'estPos', [], 'refPos', [], 'originEcef', [], 'Ce2n', []);
end


function out = run_one_case(imu, gps, Ins, caseSpec)
Nimu = size(imu.accel, 1);
Ngps = size(gps.gpsPosEcef, 1);

Ini.pos = [0; 0; 0];
Ini.vel = [0; 0; 0];

lat0 = deg2rad(double(gps.gpsPosLla(1,1)));
lon0 = deg2rad(double(gps.gpsPosLla(1,2)));
Ce2nYaw = [ ...
    -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),  cos(lat0); ...
    -sin(lon0),            cos(lon0),             0; ...
    -cos(lat0)*cos(lon0), -cos(lat0)*sin(lon0), -sin(lat0)];

alignEndTime = double(imu.timeGpsTow(1)) + 300;
gpsVelNed = (Ce2nYaw * double(gps.gpsVelEcef).').';
speed = hypot(gpsVelNed(:,1), gpsVelNed(:,2));
movingIdx = find(double(gps.timeGpsTow(:)) > alignEndTime & speed > 0.5);
nYaw = min(20, length(movingIdx));
yawIdx = movingIdx(1:nYaw);
Ini.Yaw = atan2(mean(gpsVelNed(yawIdx,2)), mean(gpsVelNed(yawIdx,1)));
Ini.att = [0; 0; Ini.Yaw];

Nav = ini_Navigation(Ini, imu, gps);
Nav = apply_tuning(Nav, caseSpec.tuning);

resultTime = nan(Nimu,1);
resultPos = nan(Nimu,3);
resultVel = nan(Nimu,3);
resultAtt = nan(Nimu,3);
resultMode = nan(Nimu,1);

while Nav.index.IMU <= Nimu
    imu_i = Nav.index.IMU;

    if Nav.mode == 0
        Nav = navi_INS_alignment(Nav, imu);
        if Nav.mode == 4
            currentTime = double(imu.timeGpsTow(imu_i));
            nextGps = find(double(gps.timeGpsTow) > currentTime, 1, 'first');
            if isempty(nextGps)
                Nav.index.GPS = Ngps + 1;
            else
                Nav.index.GPS = nextGps;
                if nextGps > 1
                    Nav.lastGpsTow = double(gps.gpsTow(nextGps-1));
                end
            end
        end
    elseif Nav.mode == 4
        Nav = navi_LC_Integ_propa_direct(Nav, imu);
        while Nav.index.GPS <= Ngps && ...
                double(gps.timeGpsTow(Nav.index.GPS)) <= double(imu.timeGpsTow(imu_i))
            gps_i = Nav.index.GPS;
            currentGpsTow = double(gps.gpsTow(gps_i));
            if currentGpsTow > Nav.lastGpsTow + 1e-6 && double(gps.gpsFix(gps_i)) >= 3
                Nav = gps_correct_for_experiment(Nav, gps, imu, caseSpec.mode);
                Nav.lastGpsTow = currentGpsTow;
            end
            Nav.index.GPS = Nav.index.GPS + 1;
        end
    end

    resultTime(imu_i) = double(imu.timeGpsTow(imu_i));
    resultPos(imu_i,:) = Nav.output.pos.';
    resultVel(imu_i,:) = Nav.output.vel.';
    resultAtt(imu_i,:) = Nav.output.att.';
    resultMode(imu_i) = Nav.mode;
    Nav.index.IMU = Nav.index.IMU + 1;
end

N = min(size(resultPos,1), size(Ins.posEcef,1));
originEcef = Nav.GPS.origin.pos_ecef.';
Ce2n = Nav.GPS.origin.C_e2n;
refPos = (Ce2n * (double(Ins.posEcef(1:N,:)) - originEcef).').';
refVel = double(Ins.velNed(1:N,:));
refYpr = double(Ins.ypr(1:N,:));
refAtt = [refYpr(:,3), refYpr(:,2), refYpr(:,1)];

estPos = resultPos(1:N,:);
estVel = resultVel(1:N,:);
estAtt = resultAtt(1:N,:) * 180/pi;
valid = resultMode(1:N) == 4 & ...
    all(isfinite(estPos),2) & all(isfinite(estVel),2) & all(isfinite(estAtt),2) & ...
    all(isfinite(refPos),2) & all(isfinite(refVel),2) & all(isfinite(refAtt),2);

time = resultTime(1:N);
firstValid = find(valid, 1, 'first');
time = time - time(firstValid);
posError = estPos - refPos;
velError = estVel - refVel;
attError = estAtt - refAtt;
attError(:,3) = atan2d(sind(attError(:,3)), cosd(attError(:,3)));

out = empty_result();
out.id = caseSpec.id;
out.label = caseSpec.label;
out.mode = caseSpec.mode;
out.posRmse = sqrt(mean(posError(valid,:).^2, 1));
out.velRmse = sqrt(mean(velError(valid,:).^2, 1));
out.attRmse = sqrt(mean(attError(valid,:).^2, 1));
out.posMax = max(abs(posError(valid,:)), [], 1);
out.velMax = max(abs(velError(valid,:)), [], 1);
out.attMax = max(abs(attError(valid,:)), [], 1);
out.finalGyroBias = Nav.imu_bias.gyro.';
out.finalAccelBias = Nav.imu_bias.acel.';

validIdx = find(valid);
keep = validIdx(1:10:end);
out.time = time(keep);
out.posError = posError(keep,:);
out.velError = velError(keep,:);
out.attError = attError(keep,:);
out.estPos = estPos(keep,:);
out.refPos = refPos(keep,:);
out.originEcef = originEcef;
out.Ce2n = Ce2n;
end


function Nav = apply_tuning(Nav, tuning)
switch tuning
    case 'P1_all_x10'
        Nav.LC.P = 10 * Nav.LC.P;
    case 'P2_all_x0p1'
        Nav.LC.P = 0.1 * Nav.LC.P;
    case 'P3_gyro_bias_x0p1'
        Nav.LC.P(13:15,13:15) = 0.1 * Nav.LC.P(13:15,13:15);
    case 'P4_gyro_bias_x0p01'
        Nav.LC.P(13:15,13:15) = 0.01 * Nav.LC.P(13:15,13:15);
    case 'P5_gyro_z_bias_x0p1'
        Nav.LC.P(15,15) = 0.1 * Nav.LC.P(15,15);
    case 'P6_gyro_z_bias_x0p01'
        Nav.LC.P(15,15) = 0.01 * Nav.LC.P(15,15);
    case 'P7_yaw_x0p1'
        Nav.LC.P(9,9) = 0.1 * Nav.LC.P(9,9);
    case 'P8_yaw_x10'
        Nav.LC.P(9,9) = 10 * Nav.LC.P(9,9);
    case 'Q1_all_x10'
        Nav.LC.Q = 10 * Nav.LC.Q;
    case 'Q2_all_x0p1'
        Nav.LC.Q = 0.1 * Nav.LC.Q;
    case 'Q3_acc_all_x10'
        Nav.LC.Q(1:3,1:3) = 10 * Nav.LC.Q(1:3,1:3);
    case 'Q4_acc_all_x0p1'
        Nav.LC.Q(1:3,1:3) = 0.1 * Nav.LC.Q(1:3,1:3);
    case 'Q5_gyro_all_x10'
        Nav.LC.Q(4:6,4:6) = 10 * Nav.LC.Q(4:6,4:6);
    case 'Q6_gyro_all_x0p1'
        Nav.LC.Q(4:6,4:6) = 0.1 * Nav.LC.Q(4:6,4:6);
    case 'Q7_acc_z_x10'
        Nav.LC.Q(3,3) = 10 * Nav.LC.Q(3,3);
    case 'Q8_gyro_z_x10'
        Nav.LC.Q(6,6) = 10 * Nav.LC.Q(6,6);
end
end


function Nav = gps_correct_for_experiment(Nav, gps, imu, mode)
gps_i = Nav.index.GPS;
imu_i = Nav.index.IMU;
pEcef = double(gps.gpsPosEcef(gps_i,:)).';
vEcef = double(gps.gpsVelEcef(gps_i,:)).';
Ce2n = Nav.GPS.origin.C_e2n;
originEcef = Nav.GPS.origin.pos_ecef;

pGpsNed = Ce2n * (pEcef - originEcef);
vGpsNed = Ce2n * vEcef;
Z = [pGpsNed; vGpsNed];
H = [eye(6), zeros(6,9)];
residual = Z - H * Nav.X;

Rgps = Nav.LC.R_GPS;
if startsWith(mode, 'adaptive')
    posSigma = double(gps.gpsPosU(gps_i,:)).';
    baseVar = diag(Rgps(1:3,1:3));
    Rgps(1:3,1:3) = diag(max(baseVar, posSigma.^2));
end

S = H * Nav.LC.P * H' + Rgps;
K = Nav.LC.P * H' / S;
if strcmp(mode, 'fixed_full_clamp')
    K(13:15,:) = 0;
elseif strcmp(mode, 'adaptive_k15')
    K(15,:) = 0;
end

Nav.X = Nav.X + K * residual;
I = eye(15);
Nav.LC.P = (I-K*H) * Nav.LC.P * (I-K*H)' + K * Rgps * K';
Nav.LC.P = 0.5 * (Nav.LC.P + Nav.LC.P');

Nav.output.pos = Nav.X(1:3);
Nav.output.vel = Nav.X(4:6);
Nav.output.att = Nav.X(7:9);
Nav.output.att(3) = atan2(sin(Nav.output.att(3)), cos(Nav.output.att(3)));
Nav.X(7:9) = Nav.output.att;
Nav.imu_bias.acel = Nav.X(10:12);
Nav.imu_bias.gyro = Nav.X(13:15);
Nav.output.C_b2n = eulr2dcm(Nav.output.att)';
Nav.output.C_n2b = Nav.output.C_b2n';

aRaw = double(imu.accel(imu_i,:)).';
wRaw = double(imu.gyro(imu_i,:)).';
fBody = aRaw - Nav.imu_bias.acel;
wBody = wRaw - Nav.imu_bias.gyro;
Nav.output.acc_ned = Nav.output.C_b2n * fBody + [0; 0; Nav.gravity];
Nav.output.pqr = wBody;
end


function metrics = results_to_table(results)
n = numel(results);
id = strings(n,1);
label = strings(n,1);
mode = strings(n,1);
values = nan(n, 24);
for k = 1:n
    id(k) = string(results(k).id);
    label(k) = string(results(k).label);
    mode(k) = string(results(k).mode);
    values(k,:) = [results(k).posRmse, results(k).velRmse, results(k).attRmse, ...
        results(k).posMax, results(k).velMax, results(k).attMax, ...
        results(k).finalGyroBias, results(k).finalAccelBias];
end
names = {'PosRMSE_N','PosRMSE_E','PosRMSE_D', ...
    'VelRMSE_N','VelRMSE_E','VelRMSE_D', ...
    'AttRMSE_Roll','AttRMSE_Pitch','AttRMSE_Yaw', ...
    'PosMax_N','PosMax_E','PosMax_D', ...
    'VelMax_N','VelMax_E','VelMax_D', ...
    'AttMax_Roll','AttMax_Pitch','AttMax_Yaw', ...
    'FinalGyroBias_X','FinalGyroBias_Y','FinalGyroBias_Z', ...
    'FinalAccelBias_X','FinalAccelBias_Y','FinalAccelBias_Z'};
values = values(:,1:numel(names));
metrics = [table(id,label,mode), array2table(values, 'VariableNames', names)];
end


function make_all_plots(results, gps, plotDir)
colors = [ ...
    0.086 0.271 0.604; ...
    0.176 0.647 0.839; ...
    0.976 0.451 0.086; ...
    0.239 0.631 0.412; ...
    0.718 0.294 0.620];

base = result_by_id(results, 'BASE');
make_trajectory(base, gps, fullfile(plotDir, 'baseline_trajectory.png'));
make_error_panel(base, 'pos', fullfile(plotDir, 'baseline_position_error.png'));
make_error_panel(base, 'vel', fullfile(plotDir, 'baseline_velocity_error.png'));
make_error_panel(base, 'att', fullfile(plotDir, 'baseline_attitude_error.png'));

make_p_yaw_plot(results, colors, fullfile(plotDir, 'p_cases_3_to_6_yaw.png'));
make_p_yaw_state_plot(results, colors, fullfile(plotDir, 'p_cases_7_to_8_yaw.png'));
make_all_case_summary(results, 'P', fullfile(plotDir, 'p_all_case_summary.png'));
make_q_acc_plot(results, colors, fullfile(plotDir, 'q_cases_3_to_4_pos_vel.png'));
make_q_gyro_plot(results, colors, fullfile(plotDir, 'q_cases_5_to_6_attitude.png'));
make_axis_specific_plot(results, 'Q7_acc_z_x10', 3, 'Down 위치 오차 [m]', ...
    fullfile(plotDir, 'q_case_7_down.png'));
make_axis_specific_plot(results, 'Q8_gyro_z_x10', 9, 'Yaw 오차 [deg]', ...
    fullfile(plotDir, 'q_case_8_yaw.png'));
make_all_case_summary(results, 'Q', fullfile(plotDir, 'q_all_case_summary.png'));
make_mitigation_plot(results, fullfile(plotDir, 'mitigation_rmse_comparison.png'));
make_mitigation_trajectories(results, fullfile(plotDir, 'mitigation_trajectories.png'));
make_gnss_quality_plot(base, gps, fullfile(plotDir, 'gnss_quality_anomaly.png'));
end


function r = result_by_id(results, id)
idx = find(strcmp({results.id}, id), 1, 'first');
r = results(idx);
end


function fig = new_figure(width, height)
fig = figure('Visible','off', 'Color','w', 'Position',[50 50 width height]);
set(fig, 'Renderer', 'painters');
end


function finish_plot(fig, outPath)
set(findall(fig, '-property', 'FontName'), 'FontName', 'Arial');
exportgraphics(fig, outPath, 'Resolution', 220, 'BackgroundColor', 'white');
close(fig);
end


function make_trajectory(base, gps, outPath)
[~, uniqueIdx] = unique(double(gps.gpsTow(:)), 'stable');
rawEcef = double(gps.gpsPosEcef(uniqueIdx,:));
rawNed = (base.Ce2n * (rawEcef - base.originEcef).').';
rawTime = double(gps.timeGpsTow(uniqueIdx));
rawFix = double(gps.gpsFix(uniqueIdx));
alignEnd = double(gps.timeGpsTow(1)) + 300;
validRaw = rawTime > alignEnd & rawFix >= 3;

fig = new_figure(1050, 760);
plot(rawNed(validRaw,2), rawNed(validRaw,1), '.', 'Color',[0.55 0.55 0.55], 'MarkerSize',4);
hold on;
plot(base.estPos(:,2), base.estPos(:,1), 'Color',[0.976 0.451 0.086], 'LineWidth',1.8);
plot(base.refPos(:,2), base.refPos(:,1), 'Color',[0.086 0.271 0.604], 'LineWidth',1.8);
grid on; axis equal;
xlabel('East [m]'); ylabel('North [m]');
title('기본 P/Q · 고정 R · K 제한 없음: 수평 궤적');
legend('Raw GNSS','INS/GNSS EKF','VN-300 Reference','Location','best');
finish_plot(fig, outPath);
end


function make_error_panel(base, kind, outPath)
switch kind
    case 'pos'
        data = base.posError; labels = {'North [m]','East [m]','Down [m]'}; titleText = 'Position Error';
    case 'vel'
        data = base.velError; labels = {'V_N [m/s]','V_E [m/s]','V_D [m/s]'}; titleText = 'Velocity Error';
    otherwise
        data = base.attError; labels = {'Roll [deg]','Pitch [deg]','Yaw [deg]'}; titleText = 'Attitude Error';
end
fig = new_figure(1100, 760);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for axisIdx = 1:3
    nexttile;
    plot(base.time, data(:,axisIdx), 'Color',[0.086 0.271 0.604], 'LineWidth',1.2);
    yline(0, ':', 'Color',[0.45 0.45 0.45]); grid on;
    ylabel(labels{axisIdx});
    if axisIdx == 3, xlabel('Time after alignment [s]'); end
end
sgtitle(sprintf('기본 조건: %s (EKF - VN-300)', titleText));
finish_plot(fig, outPath);
end


function make_p_yaw_plot(results, colors, outPath)
ids = {'BASE','P3_gyro_bias_x0p1','P4_gyro_bias_x0p01','P5_gyro_z_bias_x0p1','P6_gyro_z_bias_x0p01'};
names = {'기본','P3 bg ×0.1','P4 bg ×0.01','P5 bg_z ×0.1','P6 bg_z ×0.01'};
fig = new_figure(1180, 720);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile;
yaw = zeros(numel(ids),1);
for k = 1:numel(ids)
    yaw(k) = result_by_id(results, ids{k}).attRmse(3);
end
bar(yaw, 'FaceColor',[0.176 0.647 0.839]); grid on;
set(gca,'XTick',1:numel(names),'XTickLabel',names,'XTickLabelRotation',20);
ylabel('Yaw RMSE [deg]'); title('P Case 3~6: Yaw RMSE');
for k = 1:numel(yaw), text(k,yaw(k)+max(yaw)*0.02,sprintf('%.2f°',yaw(k)),'HorizontalAlignment','center'); end

nexttile; hold on;
for k = 1:numel(ids)
    r = result_by_id(results, ids{k});
    plot(r.time, r.attError(:,3), 'LineWidth',1.15, 'Color',colors(k,:));
end
yline(0, ':', 'Color',[0.35 0.35 0.35]); grid on;
xlabel('Time after alignment [s]'); ylabel('Yaw error [deg]');
title('시간에 따른 Yaw 오차'); legend(names,'Location','best');
finish_plot(fig, outPath);
end


function make_p_yaw_state_plot(results, colors, outPath)
ids = {'BASE','P7_yaw_x0p1','P8_yaw_x10'};
names = {'기본','P7 P_ψ ×0.1','P8 P_ψ ×10'};
fig = new_figure(1150, 720);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile;
yaw = arrayfun(@(k) result_by_id(results,ids{k}).attRmse(3), 1:numel(ids));
bar(yaw, 'FaceColor',[0.976 0.451 0.086]); grid on;
set(gca,'XTick',1:numel(names),'XTickLabel',names,'XTickLabelRotation',18);
ylabel('Yaw RMSE [deg]'); title('초기 Yaw 공분산 변화');
for k = 1:numel(yaw), text(k,yaw(k)+max(yaw)*0.02,sprintf('%.2f°',yaw(k)),'HorizontalAlignment','center'); end
nexttile; hold on;
for k = 1:numel(ids)
    r = result_by_id(results,ids{k});
    plot(r.time,r.attError(:,3),'LineWidth',1.2,'Color',colors(k,:));
end
grid on; yline(0,':'); xlabel('Time after alignment [s]'); ylabel('Yaw error [deg]');
title('Yaw 오차 시계열'); legend(names,'Location','best');
finish_plot(fig,outPath);
end


function make_q_acc_plot(results, colors, outPath)
ids = {'BASE','Q3_acc_all_x10','Q4_acc_all_x0p1'};
names = {'기본','Q3 가속도 ×10','Q4 가속도 ×0.1'};
pos = zeros(3,3); vel = zeros(3,3);
for k = 1:3
    r = result_by_id(results,ids{k}); pos(k,:) = r.posRmse; vel(k,:) = r.velRmse;
end
fig = new_figure(1180,720);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; b=bar(pos.'); grid on; title('위치 RMSE 변화'); ylabel('RMSE [m]');
set(gca,'XTickLabel',{'North','East','Down'}); legend(names,'Location','best');
for k=1:numel(b), b(k).FaceColor=colors(k,:); end
nexttile; b=bar(vel.'); grid on; title('속도 RMSE 변화'); ylabel('RMSE [m/s]');
set(gca,'XTickLabel',{'North','East','Down'}); legend(names,'Location','best');
for k=1:numel(b), b(k).FaceColor=colors(k,:); end
finish_plot(fig,outPath);
end


function make_q_gyro_plot(results, colors, outPath)
ids = {'BASE','Q5_gyro_all_x10','Q6_gyro_all_x0p1'};
names = {'기본','Q5 자이로 ×10','Q6 자이로 ×0.1'};
att = zeros(3,3);
for k=1:3, att(k,:)=result_by_id(results,ids{k}).attRmse; end
fig = new_figure(1180,720);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; b=bar(att.'); grid on; title('Roll / Pitch / Yaw RMSE'); ylabel('RMSE [deg]');
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'}); legend(names,'Location','best');
for k=1:numel(b), b(k).FaceColor=colors(k,:); end
nexttile; hold on;
for k=1:3
    r=result_by_id(results,ids{k}); plot(r.time,r.attError(:,3),'LineWidth',1.2,'Color',colors(k,:));
end
yline(0,':'); grid on; xlabel('Time after alignment [s]'); ylabel('Yaw error [deg]');
title('Yaw 오차 시계열'); legend(names,'Location','best');
finish_plot(fig,outPath);
end


function make_axis_specific_plot(results, caseId, metricIndex, ylabelText, outPath)
base = result_by_id(results,'BASE'); test = result_by_id(results,caseId);
if metricIndex <= 3
    baseData = base.posError(:,metricIndex); testData = test.posError(:,metricIndex);
    rmse = [base.posRmse(metricIndex), test.posRmse(metricIndex)];
else
    idx = metricIndex - 6;
    baseData = base.attError(:,idx); testData = test.attError(:,idx);
    rmse = [base.attRmse(idx), test.attRmse(idx)];
end
fig=new_figure(1150,720); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(base.time,baseData,'LineWidth',1.15); hold on; plot(test.time,testData,'LineWidth',1.15);
yline(0,':'); grid on; xlabel('Time after alignment [s]'); ylabel(ylabelText); title('오차 시계열'); legend('기본',test.label,'Location','best');
nexttile; bar(rmse,'FaceColor',[0.176 0.647 0.839]); grid on; set(gca,'XTickLabel',{'기본','해당 Case'});
ylabel(strrep(ylabelText,'오차','RMSE')); title('RMSE 비교');
for k=1:2, text(k,rmse(k)+max(rmse)*0.03,sprintf('%.3f',rmse(k)),'HorizontalAlignment','center'); end
finish_plot(fig,outPath);
end


function make_all_case_summary(results, prefix, outPath)
if strcmp(prefix,'P')
    ids = {'BASE','P1_all_x10','P2_all_x0p1','P3_gyro_bias_x0p1','P4_gyro_bias_x0p01','P5_gyro_z_bias_x0p1','P6_gyro_z_bias_x0p01','P7_yaw_x0p1','P8_yaw_x10'};
    names = {'Base','P1','P2','P3','P4','P5','P6','P7','P8'};
else
    ids = {'BASE','Q1_all_x10','Q2_all_x0p1','Q3_acc_all_x10','Q4_acc_all_x0p1','Q5_gyro_all_x10','Q6_gyro_all_x0p1','Q7_acc_z_x10','Q8_gyro_z_x10'};
    names = {'Base','Q1','Q2','Q3','Q4','Q5','Q6','Q7','Q8'};
end
posHorizontal = zeros(1,numel(ids)); velHorizontal=zeros(1,numel(ids)); yaw=zeros(1,numel(ids));
for k=1:numel(ids)
    r=result_by_id(results,ids{k});
    posHorizontal(k)=norm(r.posRmse(1:2));
    velHorizontal(k)=norm(r.velRmse(1:2));
    yaw(k)=r.attRmse(3);
end
fig=new_figure(1240,720); tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
nexttile; bar(posHorizontal,'FaceColor',[0.086 0.271 0.604]); grid on; ylabel('Horizontal position RMSE [m]'); title('수평 위치'); set(gca,'XTick',1:numel(names),'XTickLabel',names);
nexttile; bar(velHorizontal,'FaceColor',[0.239 0.631 0.412]); grid on; ylabel('Horizontal velocity RMSE [m/s]'); title('수평 속도'); set(gca,'XTick',1:numel(names),'XTickLabel',names);
nexttile; bar(yaw,'FaceColor',[0.976 0.451 0.086]); grid on; ylabel('Yaw RMSE [deg]'); title('Yaw'); set(gca,'XTick',1:numel(names),'XTickLabel',names);
sgtitle(sprintf('%s 튜닝 전체 Case 요약',prefix)); finish_plot(fig,outPath);
end


function make_mitigation_plot(results, outPath)
ids={'BASE','R_adaptive_no_clamp','R_adaptive_K15_zero','R_fixed_full_Kgyro_zero'};
names={'고정 R\n제한 없음','Adaptive R\n제한 없음','Adaptive R\nK_{15}=0','고정 R\nK_{13:15}=0'};
metrics=zeros(4,4);
for k=1:4
    r=result_by_id(results,ids{k}); metrics(k,:)=[r.posRmse, r.attRmse(3)];
end
fig=new_figure(1200,720); b=bar(metrics.'); grid on;
set(gca,'XTickLabel',{'Pos N [m]','Pos E [m]','Pos D [m]','Yaw [deg]'});
ylabel('RMSE'); title('R 조정과 자이로 바이어스 Kalman gain 제한 비교');
legend(names,'Location','northoutside','NumColumns',4);
colors=[0.55 0.55 0.55;0.176 0.647 0.839;0.976 0.451 0.086;0.086 0.271 0.604];
for k=1:numel(b), b(k).FaceColor=colors(k,:); end
finish_plot(fig,outPath);
end


function make_mitigation_trajectories(results, outPath)
ids={'BASE','R_adaptive_no_clamp','R_adaptive_K15_zero','R_fixed_full_Kgyro_zero'};
titles={'고정 R · 제한 없음','Adaptive R · 제한 없음','Adaptive R · K(15,:)=0','고정 R · K(13:15,:)=0'};
fig=new_figure(1180,820); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for k=1:4
    r=result_by_id(results,ids{k}); nexttile;
    plot(r.refPos(:,2),r.refPos(:,1),'Color',[0.086 0.271 0.604],'LineWidth',1.4); hold on;
    plot(r.estPos(:,2),r.estPos(:,1),'Color',[0.976 0.451 0.086],'LineWidth',1.2);
    grid on; axis equal; xlabel('E [m]'); ylabel('N [m]'); title(titles{k});
end
legend('VN-300','EKF','Location','southoutside','Orientation','horizontal');
finish_plot(fig,outPath);
end


function make_gnss_quality_plot(base, gps, outPath)
[~, uniqueIdx] = unique(double(gps.gpsTow(:)), 'stable');
gpsTime = double(gps.timeGpsTow(uniqueIdx));
gpsTime = gpsTime - (double(gps.timeGpsTow(1)) + 300);
gpsEcef = double(gps.gpsPosEcef(uniqueIdx,:));
gpsNed = (base.Ce2n * (gpsEcef - base.originEcef).').';
refN = interp1(base.time, base.refPos(:,1), gpsTime, 'linear', nan);
refE = interp1(base.time, base.refPos(:,2), gpsTime, 'linear', nan);
horizontalError = hypot(gpsNed(:,1)-refN, gpsNed(:,2)-refE);
posU = double(gps.gpsPosU(uniqueIdx,:));
sat = double(gps.gpsNumSats(uniqueIdx));
window = gpsTime >= 430 & gpsTime <= 530;

fig=new_figure(1160,760); tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(gpsTime(window),horizontalError(window),'Color',[0.976 0.451 0.086],'LineWidth',1.4); grid on; ylabel('GNSS horizontal error [m]'); title('약 483 s 구간의 GNSS 품질 저하');
nexttile; plot(gpsTime(window),posU(window,:),'LineWidth',1.1); grid on; ylabel('gpsPosU σ [m]'); legend('N','E','D','Location','best');
nexttile; stairs(gpsTime(window),sat(window),'Color',[0.086 0.271 0.604],'LineWidth',1.3); grid on; ylabel('Satellites'); xlabel('Time after alignment [s]');
finish_plot(fig,outPath);
end
