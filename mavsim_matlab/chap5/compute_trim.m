function [x_trim, u_trim] = compute_trim(mav, Va, gamma, MAV)
    % Va is the desired airspeed (m/s)
    % gamma is the desired flight path angle (radians)
    % R is the desired radius (m) - use (+) for right handed orbit, 
    %                                   (-) for left handed orbit

    % define initial state and input
    addpath('../tools');
    state0 = 
    delta0 = 
    x0 = [ state0; delta0 ];
    xstar = fmincon(@trim_objective, x0, [], [],...
                    [], [], [], [], @trim_constraints, [],...
                    mav, Va, gamma, MAV);
    x_trim = xstar(1:13);
    u_trim = xstar(14:17);
    J = trim_objective(xstar, mav, Va, gamma, MAV)
end

% objective function to be minimized
function J = trim_objective(x, mav, Va, gamma, MAV)
end

% nonlinear constraints for trim optimization
function [c, ceq] = trim_constraints(x, mav, Va, gamma, MAV)
end


