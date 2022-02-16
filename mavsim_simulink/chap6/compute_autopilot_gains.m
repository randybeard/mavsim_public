addpath('../chap5')
load transfer_function_coef
addpath('../parameters')
simulation_parameters

% AP stands for autopilot
AP.gravity = 
AP.sigma = 
AP.Va0 = 
AP.Ts = 

%----------roll loop-------------
AP.roll_kp = 
AP.roll_kd = 

%----------course loop-------------
AP.course_kp = 
AP.course_ki = 

%----------sideslip loop-------------
AP.sideslip_ki = 
AP.sideslip_kp = 

%----------yaw damper-------------
AP.yaw_damper_tau_r = 
AP.yaw_damper_kp = 

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
