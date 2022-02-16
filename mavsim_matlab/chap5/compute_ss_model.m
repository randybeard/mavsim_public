function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(mav, x_trim, u_trim, MAV)
end

% convert state x with attitude represented by quaternion
% to x_euler with attitude represented by Euler angles
function x_euler = euler_state(x_quat)
end

% convert state x_euler with attitude represented by Euler angles
% to x_quat with attitude represented by quaternions
function x_quat = quaternion_state(x_euler)
end

% return 12x1 dynamics (as if state were Euler state)
% compute f at euler_state
function xdot = f_euler(mav, x_euler, input, MAV)
end

% take partial of f_euler with respect to x_euler
function A = df_dx(mav, x_euler, input, MAV)
end

% take partial of f_euler with respect to delta
function B = df_du(mav, x_euler, delta, MAV)
end
  
  