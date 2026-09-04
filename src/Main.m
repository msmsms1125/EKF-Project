close all
clear
clc

sourceDir = fileparts(mfilename('fullpath'));
repositoryRoot = fileparts(sourceDir);
dataDir = fullfile(repositoryRoot, 'data');

addpath(fullfile(sourceDir, 'geometric'));
addpath(fullfile(sourceDir, 'navigation'));
addpath(fullfile(sourceDir, 'INSToolbox'));

load(fullfile(dataDir, 'Sens_data.mat'));
load(fullfile(dataDir, 'True_data.mat'));

Nimu = size(imu.accel,1);
Ngps = size(gps.gpsPosEcef,1);

fprintf('IMU sample 수\n');
fprintf('%d\n\n', Nimu);

fprintf('GPS sample 수\n');
fprintf('%d\n\n', Ngps);


%%%%% Navigation setting %%%%%

Ini.pos = [0;0;0];
Ini.vel = [0;0;0];

%%%%% Initial Yaw %%%%%

lat0 = deg2rad(double(gps.gpsPosLla(1,1)));
lon0 = deg2rad(double(gps.gpsPosLla(1,2)));

C_e2n_yaw = [ ...
    -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),  cos(lat0);
    -sin(lon0),            cos(lon0),             0;
    -cos(lat0)*cos(lon0), -cos(lat0)*sin(lon0), -sin(lat0)];

alignEndTime = double(imu.timeGpsTow(1)) + 300;

gpsVelEcef = double(gps.gpsVelEcef);
gpsVelNed = (C_e2n_yaw * gpsVelEcef.').';

speed = sqrt(gpsVelNed(:,1).^2 + gpsVelNed(:,2).^2);

movingIdx = find( ...
    double(gps.timeGpsTow(:)) > alignEndTime & ...
    speed > 0.5);

nYaw = min(20,length(movingIdx));
yawIdx = movingIdx(1:nYaw);

Ini.Yaw = atan2( ...
    mean(gpsVelNed(yawIdx,2)), ...
    mean(gpsVelNed(yawIdx,1)));

fprintf('Initial Yaw from GNSS COG = %.4f deg\n', ...
    Ini.Yaw*180/pi);

Ini.att = [0;0;Ini.Yaw];

Nav = ini_Navigation(Ini, imu, gps);

clear Ini


%%%%% Result allocation %%%%%

Result.time = nan(Nimu,1);
Result.pos = nan(Nimu,3);
Result.vel = nan(Nimu,3);
Result.att = nan(Nimu,3);
Result.mode = nan(Nimu,1);


%%%%% Navigation %%%%%%%%%%%%

tic

while Nav.index.IMU <= Nimu

    imu_i = Nav.index.IMU;

    %%%%% INS Alignment %%%%%
    if Nav.mode == 0

        Nav = navi_INS_alignment(Nav, imu);

        if Nav.mode == 4

            currentTime = double(imu.timeGpsTow(imu_i));

            nextGps = find( ...
                double(gps.timeGpsTow) > currentTime, ...
                1, 'first');

            if isempty(nextGps)
                Nav.index.GPS = Ngps + 1;
            else
                Nav.index.GPS = nextGps;

                if nextGps > 1
                    Nav.lastGpsTow = double(gps.gpsTow(nextGps-1));
                end
            end
        end


    %%%%% INS/GNSS Integration %%%%%
    elseif Nav.mode == 4

        Nav = navi_LC_Integ_propa_direct(Nav, imu);

        while Nav.index.GPS <= Ngps && ...
              double(gps.timeGpsTow(Nav.index.GPS)) <= ...
              double(imu.timeGpsTow(imu_i))

            gps_i = Nav.index.GPS;
            currentGpsTow = double(gps.gpsTow(gps_i));

            if currentGpsTow > Nav.lastGpsTow + 1e-6 && ...
               double(gps.gpsFix(gps_i)) >= 3

                Nav = navi_LC_Integ_GPS_correct_direct(Nav, gps, imu);
                Nav.lastGpsTow = currentGpsTow;
            end

            Nav.index.GPS = Nav.index.GPS + 1;
        end
    end


    Result.time(imu_i) = double(imu.timeGpsTow(imu_i));
    Result.pos(imu_i,:) = Nav.output.pos.';
    Result.vel(imu_i,:) = Nav.output.vel.';
    Result.att(imu_i,:) = Nav.output.att.';
    Result.mode(imu_i) = Nav.mode;

    Nav.index.IMU = Nav.index.IMU + 1;
end

elapsedTime = toc;

fprintf('\nNavigation 계산 시간 = %.4f sec\n', elapsedTime);


%%%%% VN-300 comparison %%%%%

N = min(size(Result.pos,1), size(Ins.posEcef,1));

origin_ecef = Nav.GPS.origin.pos_ecef.';
C_e2n = Nav.GPS.origin.C_e2n;

ref_pos_ecef = double(Ins.posEcef(1:N,:));
ref_pos_ned = (C_e2n * (ref_pos_ecef - origin_ecef).').';

ref_vel_ned = double(Ins.velNed(1:N,:));

ref_ypr = double(Ins.ypr(1:N,:));
ref_att_deg = [ref_ypr(:,3), ref_ypr(:,2), ref_ypr(:,1)];

est_pos = Result.pos(1:N,:);
est_vel = Result.vel(1:N,:);
est_att_deg = Result.att(1:N,:) * 180/pi;

valid = Result.mode(1:N) == 4 & ...
        all(isfinite(est_pos),2) & ...
        all(isfinite(est_vel),2) & ...
        all(isfinite(est_att_deg),2) & ...
        all(isfinite(ref_pos_ned),2) & ...
        all(isfinite(ref_vel_ned),2) & ...
        all(isfinite(ref_att_deg),2);

t = double(Result.time(1:N));
firstValid = find(valid,1,'first');
t = t - t(firstValid);

pos_error = est_pos - ref_pos_ned;
vel_error = est_vel - ref_vel_ned;
att_error = est_att_deg - ref_att_deg;

att_error(:,3) = atan2d( ...
    sind(att_error(:,3)), ...
    cosd(att_error(:,3)));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% RMSE / Max Error %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pos_rmse = sqrt(mean(pos_error(valid,:).^2,1));
vel_rmse = sqrt(mean(vel_error(valid,:).^2,1));
att_rmse = sqrt(mean(att_error(valid,:).^2,1));

pos_max = max(abs(pos_error(valid,:)),[],1);
vel_max = max(abs(vel_error(valid,:)),[],1);

fprintf('\nPosition RMSE [m]\n');
fprintf('North : %.4f\n', pos_rmse(1));
fprintf('East  : %.4f\n', pos_rmse(2));
fprintf('Down  : %.4f\n', pos_rmse(3));

fprintf('\nVelocity RMSE [m/s]\n');
fprintf('North : %.4f\n', vel_rmse(1));
fprintf('East  : %.4f\n', vel_rmse(2));
fprintf('Down  : %.4f\n', vel_rmse(3));

fprintf('\nAttitude RMSE [deg]\n');
fprintf('Roll  : %.4f\n', att_rmse(1));
fprintf('Pitch : %.4f\n', att_rmse(2));
fprintf('Yaw   : %.4f\n', att_rmse(3));

fprintf('\nPosition Max Error [m]\n');
fprintf('North : %.4f\n', pos_max(1));
fprintf('East  : %.4f\n', pos_max(2));
fprintf('Down  : %.4f\n', pos_max(3));

fprintf('\nVelocity Max Error [m/s]\n');
fprintf('North : %.4f\n', vel_max(1));
fprintf('East  : %.4f\n', vel_max(2));
fprintf('Down  : %.4f\n', vel_max(3));


%%%%% 2D Trajectory %%%%%%%%%

[~, gpsUniqueIdx] = unique(double(gps.gpsTow(:)), 'stable');

raw_gps_ecef = double(gps.gpsPosEcef(gpsUniqueIdx,:));
raw_gps_ned = (C_e2n * (raw_gps_ecef - origin_ecef).').';

raw_gps_time = double(gps.timeGpsTow(gpsUniqueIdx));
raw_gps_fix = double(gps.gpsFix(gpsUniqueIdx));

validRawGps = raw_gps_time > alignEndTime & raw_gps_fix >= 3;

figure;

plot( ...
    raw_gps_ned(validRawGps,2), ...
    raw_gps_ned(validRawGps,1), '.');

hold on;

plot( ...
    est_pos(valid,2), ...
    est_pos(valid,1), ...
    'LineWidth',1.2);

plot( ...
    ref_pos_ned(valid,2), ...
    ref_pos_ned(valid,1), ...
    'LineWidth',1.2);

grid on;
axis equal;

xlabel('East [m]');
ylabel('North [m]');
title('260212 Trajectory Comparison (VN-300)');

legend( ...
    'Raw GNSS', ...
    'INS/GNSS EKF', ...
    'VN-300 Reference', ...
    'Location','best');


%%%%% Position Error %%%%%%%%

figure;

subplot(3,1,1);
plot(t(valid), pos_error(valid,1));
grid on;
ylabel('North Error [m]');

subplot(3,1,2);
plot(t(valid), pos_error(valid,2));
grid on;
ylabel('East Error [m]');

subplot(3,1,3);
plot(t(valid), pos_error(valid,3));
grid on;
ylabel('Down Error [m]');
xlabel('Time after Alignment [s]');

sgtitle('Position Error (EKF - VN-300)');


%%%%% Velocity Error %%%%%%%%

figure;

subplot(3,1,1);
plot(t(valid), vel_error(valid,1));
grid on;
ylabel('V_N Error [m/s]');

subplot(3,1,2);
plot(t(valid), vel_error(valid,2));
grid on;
ylabel('V_E Error [m/s]');

subplot(3,1,3);
plot(t(valid), vel_error(valid,3));
grid on;
ylabel('V_D Error [m/s]');
xlabel('Time after Alignment [s]');

sgtitle('Velocity Error (EKF - VN-300)');

%%%%% Attitude Error %%%%%%%%

figure;

subplot(3,1,1);
plot(t(valid), att_error(valid,1));
grid on;
ylabel('Roll Error [deg]');

subplot(3,1,2);
plot(t(valid), att_error(valid,2));
grid on;
ylabel('Pitch Error [deg]');

subplot(3,1,3);
plot(t(valid), att_error(valid,3));
grid on;
ylabel('Yaw Error [deg]');
xlabel('Time after Alignment [s]');

sgtitle('Attitude Error (EKF - VN-300)');
