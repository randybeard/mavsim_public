% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RWB 
%   5/14/2010 - RWB
%   2/18/2019 - RWB

function y = sensors(uu, MAV, SENSOR)

    % relabel the inputs
    pn      = uu(1);
    pe      = uu(2);
    pd      = uu(3);
    u       = uu(4);
    v       = uu(5);
    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
    M_l     = uu(16);
    M_m     = uu(17);
    M_n     = uu(18);
    Va      = uu(19);
    alpha   = uu(20);
    beta    = uu(21);
    wn      = uu(22);
    we      = uu(23);
    wd      = uu(24);
    
    y_gyro_x = 
    y_gyro_y = 
    y_gyro_z = 
    y_accel_x = 
    y_accel_y = 
    y_accel_z = 
    y_static_pres = 
    y_diff_pres = 

    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



