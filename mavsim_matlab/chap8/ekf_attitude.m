% ekf_attitude
% simple ekf for estimating roll and pitch
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef ekf_attitude < handle
   %--------------------------------
    properties
        Q
        Q_gyro
        R_accel
        N
        xhat
        P
        Ts
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ekf_attitude(SENSOR, SIM)
            % load SIM and SENSOR parameters
            run('../parameters/simulation_parameters')
            run('../parameters/sensor_parameters')
            self.Q = 
            self.Q_gyro = 
            self.R_accel = 
            self.N =   % number of prediction step per sample
            self.xhat =   % initial state: phi, theta
            self.P = 
            self.Ts = SIM.ts_control/self.N;
        end
        %------methods-----------
        function state = update(self, state, measurement)
            self.propagate_model(state);
            self.measurement_update(state, measurement);
            state.phi = self.xhat(1);
            state.theta = self.xhat(2);
        end
        function f_ = f(self, x, state)
            f_ = 
        end
        function h_ = h(self, x, state)
            h_ = 
        end
        function self = propagate_model(self, state)
            % model propagation
            for i=1:self.N
                % propagate model
                self.xhat = 
                % compute Jacobian
                A = self.jacobian(@self.f, self.xhat, state);
                % compute G matrix for gyro noise
                G = 
                % convert to discrete time models
                A_d = 
                G_d = 
                % update P with discrete time model
                self.P = 
            end
        end
        function self = measurement_update(self, state, measurement)
        % measurement updates
            threshold = 2.0;
            h = self.h(self.xhat, state);
            C = self.jacobian(@self.h, self.xhat, state);
            y = [measurement.accel_x; measurement.accel_y; measurement.accel_z];
            for i=1:3
                if abs(y(i)-h(i)) < threshold
                    Ci = 
                    L = 
                    self.P = 
                    self.xhat = 
                end
            end
        end
        function J = jacobian(self, fun, x, state)
            % compute jacobian of fun with respect to x
            f = fun(x, state);
            m = size(f, 1);
            n = size(x, 1);
            eps = 0.01;  % deviation
            J = zeros(m, n);
            for i=1:n
                x_eps = x;
                x_eps(i) = x_eps(i) + eps;
                f_eps = fun(x_eps, state);
                df = (f_eps - f) / eps;
                J(:, i) = df;
            end
        end
    end
end