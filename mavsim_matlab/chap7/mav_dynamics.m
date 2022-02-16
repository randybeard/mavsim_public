% mav dynamics - implement rigid body dynamics for mav
%
% mavsimMatlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/16/2019 - RWB
classdef mav_dynamics < handle
   %--------------------------------
    properties
        ts_simulation
        state
        Va
        alpha
        beta
        wind
        true_state
        sensors
        forces
        gps_eta_n
        gps_eta_e
        gps_eta_h
        t_gps
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            addpath('../message_types'); 
            self.forces = [0; 0; 0];
            self.sensors = msg_sensors();
            self.gps_eta_n = 0;
            self.gps_eta_e = 0;
            self.gps_eta_h = 0;
            self.t_gps = 999;
        end
        %---------------------------
        function self=update_state(self, delta, wind, MAV)
        end
        %---------------------------
        function self=update_sensors(self, MAV, SENSOR)
            % Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS
            self.sensors.gyro_x = 
            self.sensors.gyro_y = 
            self.sensors.gyro_z = 
            self.sensors.accel_x = 
            self.sensors.accel_y =                
            self.sensors.accel_z = 
            self.sensors.static_pressure = 
            self.sensors.diff_pressure = 
            if self.t_gps >= SENSOR.ts_gps
                self.gps_eta_n = 
                self.gps_eta_e = 
                self.gps_eta_h = 
                self.sensors.gps_n = 
                self.sensors.gps_e = 
                self.sensors.gps_h = 
                self.sensors.gps_Vg = 
                self.sensors.gps_course = 
                self.t_gps = 0;
            else
                self.t_gps = self.t_gps + self.ts_simulation;
            end
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
            
            self.forces = Force;
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_true_state(self)
        end
    end
end