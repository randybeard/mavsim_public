function xhat = true_states(uu)
%
% fake state estimation for mavsim
%   - this function will be replaced with a state estimator in a later
%   chapter.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
%   bxhat    - estimate of x-gyro bias
%   byhat    - estimate of y-gyro bias
%   bzhat    - estimate of z-gyro bias
% 
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = -uu(3+NN); % altitude
%    u        = uu(4+NN);  % inertial velocity along body x-axis
%    v        = uu(5+NN);  % inertial velocity along body y-axis
%    w        = uu(6+NN);  % inertial velocity along body z-axis
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % yaw angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    NN = NN+12;
    Va       = uu(1+NN);  % airspeed
    alpha    = uu(2+NN);  % angle of attack
    beta     = uu(3+NN);  % sideslip angle
    wn       = uu(4+NN);  % wind North
    we       = uu(5+NN);  % wind East
%    wd       = uu(6+NN);  % wind down
    NN = NN+6;
%    t        = uu(1+NN);   % time
    
    % estimate states (using real state data)
    pnhat    = pn;
    pehat    = pe;
    hhat     = h;
    Vahat    = Va;
    alphahat = alpha;
    betahat  = beta;
    phihat   = phi;
    thetahat = theta;
    chihat   = atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
    phat     = p;
    qhat     = q;
    rhat     = r;
    Vghat    = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
    wnhat    = wn;
    wehat    = we;
    psihat   = psi;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
    
end 