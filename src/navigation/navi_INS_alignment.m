function Nav = navi_INS_alignment(Nav, imu)



i = Nav.index.IMU;

currentTime = double(imu.timeGpsTow(i));

elapsed = currentTime - Nav.imu_align.startTime;


% 실제 데이터는 N x 3
acc = double(imu.accel(i,:)).';
gyr = double(imu.gyro(i,:)).';


Nav.imu_align.sumAccel = ...
    Nav.imu_align.sumAccel + acc;

Nav.imu_align.sumGyro = ...
    Nav.imu_align.sumGyro + gyr;

Nav.imu_align.count = ...
    Nav.imu_align.count + 1;


if elapsed >= Nav.imu_align.gyroBiasStart

    Nav.imu_align.sumGyroBias = ...
        Nav.imu_align.sumGyroBias + gyr;

    Nav.imu_align.countGyroBias = ...
        Nav.imu_align.countGyroBias + 1;

end

if elapsed >= Nav.imu_align.t_size


    meanAccel = ...
        Nav.imu_align.sumAccel ...
        / Nav.imu_align.count;


    meanGyroBias = ...
        Nav.imu_align.sumGyroBias ...
        / Nav.imu_align.countGyroBias;


    Nav.imu_bias.gyro = meanGyroBias;

    Nav.X(13:15) = Nav.imu_bias.gyro;

    Nav.gravity = norm(meanAccel);


    roll0 = atan2( ...
        -meanAccel(2), ...
        -meanAccel(3));



    value = meanAccel(1) / Nav.gravity;

    value = max(min(value,1),-1);

    pitch0 = asin(value);



    yaw0 = Nav.output.att(3);


    Nav.output.att = [ ...
        roll0;
        pitch0;
        yaw0];


    Nav.X(7:9) = Nav.output.att;



    Nav.output.C_b2n = ...
        eulr2dcm(Nav.output.att)';

    Nav.output.C_n2b = ...
        Nav.output.C_b2n';



    Nav.mode = 4;


    % 출력

    fprintf('\n====== Alignment Complete ======\n');

    fprintf('Gravity = %.6f m/s^2\n', ...
        Nav.gravity);

    fprintf('Roll  = %.4f deg\n', ...
        roll0*180/pi);

    fprintf('Pitch = %.4f deg\n', ...
        pitch0*180/pi);

    fprintf('Yaw   = %.4f deg\n', ...
        yaw0*180/pi);


    fprintf('\nGyro Bias - Last 60 sec [rad/s]\n');

    disp(Nav.imu_bias.gyro.');


    fprintf('Gyro Bias [deg/s]\n');

    disp((Nav.imu_bias.gyro.' * 180/pi));


    fprintf('================================\n\n');

end


end
