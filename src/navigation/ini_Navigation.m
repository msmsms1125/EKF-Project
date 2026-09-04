function Nav = ini_Navigation(Ini, imu, gps)

Nav.output.pos = Ini.pos;
Nav.output.vel = Ini.vel;
Nav.output.att = Ini.att;

Nav.output.acc_ned = zeros(3,1);
Nav.output.pqr     = zeros(3,1);

Nav.output.C_b2n = eulr2dcm(Nav.output.att)';
Nav.output.C_n2b = Nav.output.C_b2n';

Nav.imu_bias.acel = zeros(3,1);
Nav.imu_bias.gyro = zeros(3,1);

Nav.gravity = 9.81;

% X =
%
% 1~3   Position NED
% 4~6   Velocity NED
% 7~9   Roll Pitch Yaw
% 10~12 Accel Bias
% 13~15 Gyro Bias

Nav.X = [Nav.output.pos;
         Nav.output.vel;
         Nav.output.att;
         Nav.imu_bias.acel;
         Nav.imu_bias.gyro];

Nav.mode = 0;

Nav.imu_align.t_size = 300;

Nav.imu_align.sumAccel = zeros(3,1);
Nav.imu_align.sumGyro  = zeros(3,1);

Nav.imu_align.count = 0;

% Gyro bias는 마지막 60초만 사용
Nav.imu_align.gyroBiasStart = 240;   % 300 - 60 sec

Nav.imu_align.sumGyroBias = zeros(3,1);
Nav.imu_align.countGyroBias = 0;

Nav.imu_align.startTime = double(imu.timeGpsTow(1));

Nav.GPS.origin.pos_ecef = double(gps.gpsPosEcef(1,:)).';


lat0 = deg2rad(double(gps.gpsPosLla(1,1)));
lon0 = deg2rad(double(gps.gpsPosLla(1,2)));

C_e2n = [ ...
    -sin(lat0)*cos(lon0), ...
    -sin(lat0)*sin(lon0), ...
     cos(lat0);

    -sin(lon0), ...
     cos(lon0), ...
     0;

    -cos(lat0)*cos(lon0), ...
    -cos(lat0)*sin(lon0), ...
    -sin(lat0)];


Nav.GPS.origin.C_e2n = C_e2n;
Nav.GPS.origin.C_n2e = C_e2n';

Nav.LC.P = zeros(15,15);


%% Position
Nav.LC.P(1:3,1:3) = (5^2) * eye(3);


%% Velocity
Nav.LC.P(4:6,4:6) = (1^2) * eye(3);


%% Attitude
Nav.LC.P(7:9,7:9) = (5*pi/180)^2 * eye(3);


%% Accelerometer Bias
Nav.LC.P(10:12,10:12) = (0.1^2) * eye(3);


%% Gyroscope Bias
Nav.LC.P(13:15,13:15) = (0.01*pi/180)^2 * eye(3);

Nav.LC.Q = zeros(6,6);


%% accelerometer noise
Nav.LC.Q(1:3,1:3) = (0.1^2) * eye(3);


%% gyro noise
Nav.LC.Q(4:6,4:6) = (0.01*pi/180)^2 * eye(3);

Nav.LC.R_GPS = zeros(6,6);


Nav.LC.R_GPS(1:3,1:3) = diag([ ...
    3^2, ...
    3^2, ...
    5^2]);


Nav.LC.R_GPS(4:6,4:6) = diag([ ...
    0.3^2, ...
    0.3^2, ...
    0.3^2]);

Nav.index.IMU = 1;
Nav.index.GPS = 1;

Nav.lastGpsTow = -inf;


end
