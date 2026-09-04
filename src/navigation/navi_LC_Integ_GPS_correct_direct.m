function Nav = navi_LC_Integ_GPS_correct_direct(Nav, gps, imu)



gps_i = Nav.index.GPS;
imu_i = Nav.index.IMU;


p_ecef = double(gps.gpsPosEcef(gps_i,:)).';

v_ecef = double(gps.gpsVelEcef(gps_i,:)).';



C_e2n = Nav.GPS.origin.C_e2n;

origin_ecef = Nav.GPS.origin.pos_ecef;


%% position

p_gps_ned = C_e2n * ...
    (p_ecef - origin_ecef);


%% velocity

v_gps_ned = C_e2n * ...
    v_ecef;


Z = [p_gps_ned;
     v_gps_ned];


H = [eye(6), zeros(6,9)];


residual = Z - H*Nav.X;


R_GPS = Nav.LC.R_GPS;

pos_sigma = double(gps.gpsPosU(gps_i,:)).';

pos_var_default = diag(R_GPS(1:3,1:3));
pos_var_reported = pos_sigma.^2;

R_GPS(1:3,1:3) = diag( ...
    max(pos_var_default, pos_var_reported));


S = H * Nav.LC.P * H' ...
  + R_GPS;


K = Nav.LC.P * H' / S;


%% GNSS 위치/속도로 Z축 자이로 바이어스가 잘못 보정되는 현상 방지
K(15,:) = 0;


Nav.X = Nav.X ...
      + K * residual;


I = eye(15);


Nav.LC.P = ...
    (I-K*H) * Nav.LC.P * (I-K*H)' ...
    + K * R_GPS * K';


Nav.LC.P = 0.5 * ...
    (Nav.LC.P + Nav.LC.P');



Nav.output.pos = Nav.X(1:3);

Nav.output.vel = Nav.X(4:6);

Nav.output.att = Nav.X(7:9);


Nav.output.att(3) = atan2( ...
    sin(Nav.output.att(3)), ...
    cos(Nav.output.att(3)));


Nav.X(7:9) = Nav.output.att;


Nav.imu_bias.acel = Nav.X(10:12);

Nav.imu_bias.gyro = Nav.X(13:15);


Nav.output.C_b2n = eulr2dcm(Nav.output.att)';

Nav.output.C_n2b = Nav.output.C_b2n';


a_raw = double(imu.accel(imu_i,:)).';
w_raw = double(imu.gyro(imu_i,:)).';


f_b = a_raw ...
    - Nav.imu_bias.acel;

w_b = w_raw ...
    - Nav.imu_bias.gyro;


f_n = Nav.output.C_b2n * f_b;


g_n = [0;
       0;
       Nav.gravity];


Nav.output.acc_ned = f_n + g_n;

Nav.output.pqr = w_b;


end
