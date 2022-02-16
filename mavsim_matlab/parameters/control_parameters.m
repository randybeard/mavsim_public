% control_parameters
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
addpath('../chap5')
load transfer_function_coef

% AP stands for autopilot
AP.gravity = 
AP.sigma = 
AP.Va0 = 

%----------roll loop-------------
AP.roll_kp = 
AP.roll_kd = 

%----------course loop-------------
AP.course_kp = 
AP.course_ki = 

%----------sideslip loop-------------
AP.sideslip_ki = 
AP.sideslip_kp = 

%----------pitch loop-------------
AP.pitch_kp = 
AP.pitch_kd = 
K_theta_DC = 

%----------altitude loop-------------
AP.altitude_kp = 
AP.altitude_ki = 
AP.altitude_zone = 

%---------airspeed hold using throttle---------------
AP.airspeed_throttle_kp = 
AP.airspeed_throttle_ki = 
