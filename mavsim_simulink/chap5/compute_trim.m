% compute trim conditions using 'mavsim_chap5_trim.slx'
% nominal airspeed P.Va0 specified above with aircraft parameters
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = 10000000000;        % desired radius (m) - use (+) for right handed orbit, 
                            %                          (-) for left handed orbit
Va = 25;


% set initial conditions 
x0 = 
% specify which states to hold equal to the initial conditions
ix = [];

% specify initial inputs 
u0 = [...
    0;... % 1 - delta_e
    0;... % 2 - delta_a
    0;... % 3 - delta_r
    1;... % 4 - delta_t
    ];
% specify which inputs to hold constant
iu = [];

% define constant outputs
y0 = [...
    Va;...       % 1 - Va
    0;...        % 2 - alpha
    0;...        % 3 - beta
    ];
% specify which outputs to hold constant
iy = [1,3];

% define constant derivatives
dx0 = 

if R~=Inf, dx0(9) = Va*cos(gamma)/R; end  % 9 - psidot
% specify which derivaties to hold constant in trim algorithm
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim('mavsim_trim',x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))

% P.u_trim = u_trim;
% P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
MAV.pn0    = 0;           % initial North position
MAV.pe0    = 0;           % initial East position
MAV.pd0    = -200;        % initial Down position (negative altitude)
MAV.u0     = x_trim(4);   % initial velocity along body x-axis
MAV.v0     = x_trim(5);   % initial velocity along body y-axis
MAV.w0     = x_trim(6);   % initial velocity along body z-axis
MAV.phi0   = x_trim(7);   % initial roll angle
MAV.theta0 = x_trim(8);   % initial pitch angle
MAV.psi0   = x_trim(9);   % initial yaw angle
MAV.p0     = x_trim(10);  % initial body frame roll rate
MAV.q0     = x_trim(11);  % initial body frame pitch rate
MAV.r0     = x_trim(12);  % initial body frame yaw rate  


