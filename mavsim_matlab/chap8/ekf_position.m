% ekf_position
% simple ekf for estimating pn, pe, chi, Vg, wn, we, psi
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/4/2019 - RWB
classdef ekf_position < handle
   %--------------------------------
    properties
        Q
        R
        N
        xhat
        P
        Ts
        gps_n_old
        gps_e_old
        gps_Vg_old
        gps_course_old
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ekf_position(SENSOR, SIM)
            % load SIM and SENSOR parameters
            run('../parameters/simulation_parameters')
            run('../parameters/sensor_parameters')
            self.Q = 
            self.R = 
            self.N =   % number of prediction step per sample
            self.Ts = (SIM.ts_control / self.N);
            self.xhat = 
            self.P = 
            self.gps_n_old = 9999;
            self.gps_e_old = 9999;
            self.gps_Vg_old = 9999;
            self.gps_course_old = 9999;
        end
        %------methods-----------
        function state = update(self, state, measurement)
            self.propagate_model(state);
            self.measurement_update(state, measurement);
            state.pn = self.xhat(1);
            state.pe = self.xhat(2);
            state.Vg = self.xhat(3);
            state.chi = self.xhat(4);
            state.wn = self.xhat(5);
            state.we = self.xhat(6);
            state.psi = self.xhat(7);
        end
        function f_ = f(self, x, state)
            f_ = 
        end
        function h = h_gps(self, x, state)
            h = 
        end
        function h = h_pseudo(self, x, state)
            % measurement model for wind triangle pseudo measuremnt
            h = 
       end
        function self = propagate_model(self, state)
            % model propagation
            for i=1:self.N
                % propagate model
                self.xhat = 
                % compute Jacobian
                A = self.jacobian(@self.f, self.xhat, state);
                % update P with continuous time model
                % self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
                % convert to discrete time models
                A_d = 
                % update P with discrete time model
                self.P = 
            end
        end
        function self = measurement_update(self, state, measurement)
            % always update based on wind triangle pseudu measurement
            h = self.h_pseudo(self.xhat, state);
            C = self.jacobian(@self.h_pseudo, self.xhat, state);
            y = 
            for i=1:2
                Ci = 
                L = 
                self.P = 
                self.xhat = 
            end

            %# only update GPS when one of the signals changes
            if (measurement.gps_n ~= self.gps_n_old)...
                || (measurement.gps_e ~= self.gps_e_old)...
                || (measurement.gps_Vg ~= self.gps_Vg_old)...
                || (measurement.gps_course ~= self.gps_course_old)

                h = self.h_gps(self.xhat, state);
                C = self.jacobian(@self.h_gps, self.xhat, state);
                y = 
                for i=1:4
                    Ci = 
                    L = 
                    self.P = 
                    self.xhat = 
                end

                % update stored GPS signals
                self.gps_n_old = measurement.gps_n;
                self.gps_e_old = measurement.gps_e;
                self.gps_Vg_old = measurement.gps_Vg;
                self.gps_course_old = measurement.gps_course;
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