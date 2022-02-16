%-------- Accelerometer --------
% standard deviation of accelerometers in m/s^2
SENSOR.accel_sigma = 0.0025*9.8; 

%-------- Rate Gyro --------
SENSOR.gyro_x_bias = 0;  % bias on x_gyro
SENSOR.gyro_y_bias = 0;  % bias on y_gyro
SENSOR.gyro_z_bias = 0;  % bias on z_gyro
% standard deviation of gyros in rad/sec
SENSOR.gyro_sigma = 0.13 * pi/180;  

%-------- Pressure Sensor(Altitude) --------
% standard deviation of static pressure sensors in Pascals
SENSOR.static_pres_sigma = 0.01*1000;  

%-------- Pressure Sensor (Airspeed) --------
% standard deviation of diff pressure sensor in Pascals
SENSOR.diff_pres_sigma = 0.002*1000;  

%-------- Magnetometer --------
SENSOR.mag_beta = 1.0 * (pi/180);
SENSOR.mag_sigma = 0.03 * (pi/180);

%-------- GPS --------
SENSOR.ts_gps = 1.0;
SENSOR.gps_beta = 1 / 1100;  % 1 / s
SENSOR.gps_n_sigma = 0.21;
SENSOR.gps_e_sigma = 0.21;
SENSOR.gps_h_sigma = 0.40;
SENSOR.gps_Vg_sigma = 0.05;
SENSOR.gps_course_sigma = SENSOR.gps_Vg_sigma / 10;
