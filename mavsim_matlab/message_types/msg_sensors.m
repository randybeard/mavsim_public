% msg_sensors
%   - message type for sensors
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         2/16/2019 - RWB
classdef msg_sensors
   %--------------------------------
    properties
        gyro_x
        gyro_y
        gyro_z
        accel_x
        accel_y
        accel_z
        static_pressure
        diff_pressure
        gps_n
        gps_e
        gps_h
        gps_Vg
        gps_course
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_sensors()
            self.gyro_x = 0;  
            self.gyro_y = 0;
            self.gyro_z = 0;
            self.accel_x = 0;
            self.accel_y = 0;
            self.accel_z = 0;
            self.static_pressure = 0;
            self.diff_pressure = 0;
            self.gps_n = 0;
            self.gps_e = 0;
            self.gps_h = 0;
            self.gps_Vg = 0;
            self.gps_course = 0;      
        end
    end
end