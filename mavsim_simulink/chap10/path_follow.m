% path follow
%  - follow straight line path or orbit
%
% Modified:
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%  phi_ff - feed forward roll command
%
function out = path_follow(in)
    % path following gains
    chi_infty =   % approach angle for large distance from straight-line path
    k_path    =         % proportional gain for path following
    k_orbit   =        % proportional gain for orbit following
    gravity   = 

    NN = 0;
    flag      = in(1+NN);
    Va_d      = in(2+NN);
    r_path    = [in(3+NN); in(4+NN); in(5+NN)];
    q_path    = [in(6+NN); in(7+NN); in(8+NN)];
    c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
    rho_orbit = in(12+NN);
    lam_orbit = in(13+NN);
    NN = NN + 13;
    pn        = in(1+NN);
    pe        = in(2+NN);
    h         = in(3+NN);
    Va        = in(4+NN);
    alpha   = in(5+NN);
    beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    p       = in(10+NN);
    q       = in(11+NN);
    r       = in(12+NN);
    Vg      = in(13+NN);
    wn      = in(14+NN);
    we      = in(15+NN);
    psi     = in(16+NN);
    NN = NN + 16;
    t         = in(1+NN);
  
    switch flag
        case 1 % follow straight line path specified by r and q
          
            % compute wrapped version of path angle
            chi_q = 
            while (chi_q - chi < -pi), chi_q = chi_q + 2*pi; end
            while (chi_q - chi > +pi), chi_q = chi_q - 2*pi; end

            % heading command
            chi_c = 

            % commanded altitude
            h_c = 
            % roll feedforward command
            phi_ff = 
           
        case 2 % follow orbit specified by c, rho, lam

            d =  % distance from orbit center
            % compute wrapped version of angular position on orbit
            varphi = 
            while (varphi - chi < -pi), varphi = varphi + 2*pi; end
            while (varphi - chi > +pi), varphi = varphi - 2*pi; end
             % compute orbit error
            orbit_error = 
            % heading command
            chi_c = 
         
            % commanded altitude is the height of the orbit
            h_c = 
                    
            % the roll angle feedforward command 
            phi_ff = 
    end
  
    % command airspeed equal to desired airspeed
    Va_c = Va_d;
  
    % create output
    out = [Va_c; h_c; chi_c; phi_ff];
end


