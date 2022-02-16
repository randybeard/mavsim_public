% path manager for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/22/2019 - RWB
classdef path_manager < handle
   %--------------------------------
    properties
        path
        halfspace_n
        halfspace_r
        ptr_current
        ptr_previous
        ptr_next
        num_waypoints
        flag_need_new_waypoints
        manager_state
        dubins_path
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_manager
            % path definition
            addpath('../message_types'); 
            self.path = msg_path(); 
            % pointers to previous, current, and next waypoints
            self.ptr_previous = 1;
            self.ptr_current = 2; 
            self.ptr_next = 3;
            % flag that request new waypoints from path planner
            self.flag_need_new_waypoints = 1; 
            % flag that indicates start of simulation
            self.num_waypoints = 0;
            self.halfspace_n = inf*[1;1;1];
            self.halfspace_r = inf*[1;1;1];
            % state of the manager state machine
            self.manager_state = 1;
            addpath('../chap11');
            self.dubins_path = dubins_parameters();
        end
        %------methods-----------
        function path = update(self, waypoints, radius, state)
            % this flag is set for one time step to signal a redraw in the
            % viewer
            if self.path.flag_path_changed == 1  
                self.path.flag_path_changed = 0;
            end
            if waypoints.num_waypoints == 0
                waypoints.flag_manager_requests_waypoints = 1;
            end
            if isequal(waypoints.type, 'straight_line')
                self.line_manager(waypoints, state);
            elseif isequal(waypoints.type, 'fillet')
                self.fillet_manager(waypoints, radius, state);
            elseif isequal(waypoints.type, 'dubins')
                self.dubins_manager(waypoints, radius, state);
            else
                disp('Error in Path Manager: Undefined waypoint type.')
            end
            % return the commanded path
            path = self.path;
        end
        %---------------------------
        function self = line_manager(self, waypoints, state)
            mav_pos = [state.pn; state.pe; -state.h];
            % this flag is set for one time step to signal a redraw in the
            % viewer
            if self.path.flag_path_changed == 1  
                self.path.flag_path_changed = 0;
            end
            if waypoints.num_waypoints == 0
                waypoints.flag_manager_requests_waypoints = 1;
            else
                % if the waypoints have changed, update the waypoint pointer
                if waypoints.flag_waypoints_changed == 1
                    waypoints.flag_manager_requests_waypoints = 0;
                    waypoints.flag_waypoints_changed = 0; 
                end
                
            end
        end
        %---------------------------
        function self = fillet_manager(self, waypoints, radius, state)
            mav_pos = [state.pn; state.pe; -state.h];
            % this flag is set for one time step to signal a redraw in the
            % viewer
            if self.path.flag_path_changed == 1  
                self.path.flag_path_changed = 0;
            end
            if waypoints.num_waypoints == 0
                waypoints.flag_manager_requests_waypoints = 1;
            else
                % if the waypoints have changed, update the waypoint pointer
                if waypoints.flag_waypoints_changed == 1
                    waypoints.flag_manager_requests_waypoints = 0;
                    waypoints.flag_waypoints_changed = 0;

                end
                
                
            end
        end
        %---------------------------
        function self = dubins_manager(self, waypoints, radius, state)
            mav_pos = [state.pn; state.pe; -state.h];
            % this flag is set for one time step to signal a redraw in the
            % viewer
            if self.path.flag_path_changed == 1  
                self.path.flag_path_changed = 0;
            end
            if waypoints.num_waypoints == 0
                waypoints.flag_manager_requests_waypoints = 1;
            else
                % if the waypoints have changed, update the waypoint pointer
                if waypoints.flag_waypoints_changed == 1
                    waypoints.flag_manager_requests_waypoints = 0;
                    waypoints.flag_waypoints_changed = 0;

                end
                
            end
        end
        %---------------------------
        function self = initialize_pointers(self)
        end   
        %---------------------------
        function self = increment_pointers(self) 
        end
        %---------------------------
        function flag = inHalfSpace(self, pos) 
            if (pos-self.halfspace_r)'*self.halfspace_n >= 0
                flag = 1;
            else
                flag = 0;
            end
        end
    end
end