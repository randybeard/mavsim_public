% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RWB 
%   5/14/2010 - RWB
%   2/18/2019 - RWB

function y = gps(uu, SENSOR)

    % relabel the inputs
    Va      = uu(1);
    alpha   = uu(2);
    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
    u       = uu(10);
    v       = uu(11);
    w       = uu(12);
    phi     = uu(13);
    theta   = uu(14);
    psi     = uu(15);
    p       = uu(16);
    q       = uu(17);
    r       = uu(18);
    t       = uu(19);
    
    y_gps_n      = 
    y_gps_e      = 
    y_gps_h      =
    y_gps_Vg     = 
    y_gps_course = 

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



