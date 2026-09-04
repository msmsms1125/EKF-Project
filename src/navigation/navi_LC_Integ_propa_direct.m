function Nav = navi_LC_Integ_propa_direct(Nav, imu)

i = Nav.index.IMU;


if i == 1

    dt = median(diff(double(imu.timeGpsTow)));

else

    dt = double(imu.timeGpsTow(i) ...
              - imu.timeGpsTow(i-1));

end


if dt <= 0 || dt > 0.1

    dt = 0.01;

end


a_raw = double(imu.accel(i,:)).';
w_raw = double(imu.gyro(i,:)).';


b_acc = Nav.imu_bias.acel;
b_gyr = Nav.imu_bias.gyro;


f_b = a_raw - b_acc;

w_b = w_raw - b_gyr;



p_ned = Nav.output.pos;
v_ned = Nav.output.vel;
att   = Nav.output.att;


phi   = att(1);
theta = att(2);
psi   = att(3);


sphi = sin(phi);
cphi = cos(phi);

stheta = sin(theta);
ctheta = cos(theta);

spsi = sin(psi);
cpsi = cos(psi);

T_eul = [ ...
    1, sphi*stheta/ctheta, cphi*stheta/ctheta;

    0, cphi,              -sphi;

    0, sphi/ctheta,        cphi/ctheta];


eul_dot = T_eul * w_b;


C_b2n = Nav.output.C_b2n;


f_n = C_b2n * f_b;



g_n = [0;
       0;
       Nav.gravity];


a_n = f_n + g_n;

p_new = p_ned ...
      + v_ned * dt ...
      + 0.5 * a_n * dt^2;

v_new = v_ned ...
      + a_n * dt;


att_new = att ...
        + eul_dot * dt;


att_new(3) = atan2( ...
    sin(att_new(3)), ...
    cos(att_new(3)));


A = zeros(15,15);


A(1:3,4:6) = eye(3);


dC_dphi = [ ...
    0, ...
    cphi*stheta*cpsi + sphi*spsi, ...
   -sphi*stheta*cpsi + cphi*spsi;

    0, ...
    cphi*stheta*spsi - sphi*cpsi, ...
   -sphi*stheta*spsi - cphi*cpsi;

    0, ...
    cphi*ctheta, ...
   -sphi*ctheta];


dC_dtheta = [ ...
   -stheta*cpsi, ...
    sphi*ctheta*cpsi, ...
    cphi*ctheta*cpsi;

   -stheta*spsi, ...
    sphi*ctheta*spsi, ...
    cphi*ctheta*spsi;

   -ctheta, ...
   -sphi*stheta, ...
   -cphi*stheta];


dC_dpsi = [ ...
   -ctheta*spsi, ...
   -sphi*stheta*spsi - cphi*cpsi, ...
   -cphi*stheta*spsi + sphi*cpsi;

    ctheta*cpsi, ...
    sphi*stheta*cpsi - cphi*spsi, ...
    cphi*stheta*cpsi + sphi*spsi;

    0, ...
    0, ...
    0];


A(4:6,7) = dC_dphi   * f_b;
A(4:6,8) = dC_dtheta * f_b;
A(4:6,9) = dC_dpsi   * f_b;


A(4:6,10:12) = -C_b2n;

dT_dphi = [ ...
    0, ...
    cphi*stheta/ctheta, ...
   -sphi*stheta/ctheta;

    0, ...
   -sphi, ...
   -cphi;

    0, ...
    cphi/ctheta, ...
   -sphi/ctheta];


dT_dtheta = [ ...
    0, ...
    sphi/(ctheta^2), ...
    cphi/(ctheta^2);

    0, ...
    0, ...
    0;

    0, ...
    sphi*stheta/(ctheta^2), ...
    cphi*stheta/(ctheta^2)];


A(7:9,7) = dT_dphi * w_b;
A(7:9,8) = dT_dtheta * w_b;


%% gyro bias -> attitude

A(7:9,13:15) = -T_eul;


B = zeros(15,6);



B(4:6,1:3) = C_b2n;



B(7:9,4:6) = T_eul;



F = eye(15) + A*dt;

G = B*dt;


Nav.LC.P = F * Nav.LC.P * F' ...
         + G * Nav.LC.Q * G';



Nav.LC.P = 0.5 * ...
    (Nav.LC.P + Nav.LC.P');


Nav.output.pos = p_new;
Nav.output.vel = v_new;
Nav.output.att = att_new;

Nav.output.acc_ned = a_n;
Nav.output.pqr = w_b;


Nav.X = [ ...
    p_new;
    v_new;
    att_new;
    Nav.imu_bias.acel;
    Nav.imu_bias.gyro];

Nav.output.C_b2n = eulr2dcm(Nav.output.att)';
Nav.output.C_n2b = Nav.output.C_b2n';


end
