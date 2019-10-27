classdef odometry_observer < handle
    %ODOMETRY_OBSERVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        P_hat = eye(3); % capital P hat
        p_hat = {};     % little p hat
        
        gaink =0.5;
        gainl =0;
        iterate = 0;
        
        c_params = [];   % maps internal numbers to external ids
                        % accept id 1 - 19 only
    end
    
    methods
        function obj = odometry_observer(state)
            obj.P_hat = [ cos(state(3)) -sin(state(3))  state(1);         % initial pose of the robot to be [x=0 y=0 theta=0]
                      sin(state(3))  cos(state(3))  state(2);
                      0              0              1 ];
            obj.p_hat = cell(1,20);    % accept id 1 - 19 only, num 20 is for 0
            obj.c_params = ones(1,20);
        end
        
        % current state yi input 
        function input_measurements(obj, dt, iterate, y, nums, se2_velocity) % se2_velocity [u q]
            
            % velocity
            u = se2_velocity(1);
            q = se2_velocity(2);
            U = [0 -q  u;
                 q  0  0;
                 0  0  0 ];
            
            % y
            count = size(nums,1);
            if count ~= 0
                y = y';
                y(3,:)=1;
            end
            
            dt = dt / iterate;
            for iterate_count = 1:iterate
                % add landmarks
                count = size(nums,1);
                for i = 1:count
                    id = nums(i);    % get id number
                    if(isempty(obj.p_hat{1,id}))   % if this id's p_hat is already initialized 
                        obj.p_hat{1,id} = obj.P_hat * y(:,i);
                    end
                end

                % update P_hat and p_hat
                count = size(nums,1);

                % get epsilon for each detected marker
                epsilon = cell(1,count);
                for i = 1:count
                    id = nums(i);    % get id number
                    epsilon{1,i} = obj.P_hat * y(:,i) - obj.p_hat{1,id};    % a set of Epsilon
                end

                % update P_hat

                % sum of ci * P_hat' * epsilon * y'
                count = size(nums,1);
                tmp = zeros(3);
                for i = 1:count
                    tmp = tmp + obj.P_hat' * epsilon{1,i} * y(:,i)';
                end
                tmp(1,1)=0;tmp(2,2:3)=0;tmp(3,:)=0;
                obj.P_hat = obj.P_hat * expm( dt * ( U - obj.gaink * tmp)); % update P_hat

                % update p_hat
                count = size(nums,1);
                for i = 1:count
                    id = nums(i);
                    obj.p_hat{1,id} = obj.p_hat{1,id} - dt * obj.gainl * obj.c_params(1,id) * epsilon{1,i};   % update p_hat
                    if obj.c_params(1,id) > 0.3
                        obj.c_params = obj.c_params - 0.3;
                    end
                end
            end
        end
        
        % output Pose and pi
        function [P, p, nums] = output_state(obj)
            
            % P 
            t = obj.P_hat(1:2,1:2)*[1;0];
            angle = atan2(t(2),t(1));
            P = [obj.P_hat(1,3),obj.P_hat(2,3),angle];
            
            % p
            p = [];
            for i = 1:20
               if ~isempty(obj.p_hat{i})
                   p = [p; obj.p_hat{i}'];
               end
            end
            
            % nums
            nums = [];
            for i = 1:20
               if ~isempty(obj.p_hat{i})
                   nums = [nums; i];
               end
            end
        end
    end
end

