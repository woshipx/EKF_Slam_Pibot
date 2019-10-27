classdef ekf_slam < handle
    %EKF_SLAM The EKF algorithm for SLAM
    
    properties  
        
        % Robot State and Covariance
        state_t = zeros(3,1); % The estimated state vector. 
                        % Initial robot state [theta;state_t;y]. 
        Cov_t = zeros(3,3); % The estimated state covariance
                        % Covariance [theta_theta theta_state_t theta_y;
                        %             theta_state_t     state_t_state_t     state_t_y    ;
                        %             theta_y     state_t_y     y_y    ]
        
        % State prediction (predict in input_velocity())
        state_t_pre = [];
        Cov_t_pre = [];
        
		% The covariance values provided here are NOT correct!
        sigxy = 0.0004; % The covariance of linear velocity
        sigth = 0.0003; % The covariance of angular velocity
        siglm = 0.01; % The covariance of landmark measurements
        
        idx2num = []; % The map from state vector index to landmark id.
                    
    end
    
    methods
        
  
        function input_velocity(obj, dt, u, q)
    [wheel_velocities] = reparam_controlL(u, q);
    [u,q] = velocity_calibration(wheel_velocities);
            % Perform the update step of the EKF. This involves updating
            % the state and covariance estimates using the input velocity,
            % the time step, and the covariance of the update step.
            
            n = size(obj.state_t,1);
            %% Covariance prediction 
 
            % At  
            At = eye(n);
            At(2,1) = -sin(obj.state_t(1))*u*dt;
            At(3,1) = cos(obj.state_t(1))*u*dt;
            
            % Bt 
            Bt = zeros(n,2);
            Bt(1,1) = dt;
            Bt(2,2) = dt * cos(obj.state_t(1));
            Bt(3,2) = dt * sin(obj.state_t(1));
            
            % Rt
            
            % covariance matristate_t for [u, q] on state [theta state_t y]
            Rt = zeros(2,2);
            Rt(1,1) = obj.sigth;    % variance on q 
            Rt(2,2) = obj.sigxy;    % variance on u 
            
            
            Rt_s = eye(n);
            Rt_s(1:3,1:3) = 0;
            Rt_s = obj.sigxy*Rt_s;            

            % P_pre 
            obj.Cov_t_pre = At * obj.Cov_t * At' + Bt * Rt * Bt' + Rt_s;
            
            %% State prediction: prior estimation for t+1
            obj.state_t_pre = zeros(n,1);    % init state_t_pre
            
            % predicte theta state_t y from q and u
            obj.state_t_pre(1) = obj.state_t(1) + dt*q;                 % theta
            obj.state_t_pre(2) = obj.state_t(2) + dt*cos(obj.state_t(1))*u;   % state_t
            obj.state_t_pre(3) = obj.state_t(3) + dt*sin(obj.state_t(1))*u;   % y
            
            obj.state_t_pre(4:n) = obj.state_t(4:n);  % exist landmarks 
            

        end
        
        function input_measurements(obj, centres, nums)
            %% Perform the innovation step of the EKF. This involves adding
            num_landmark = size(nums,1);
            theta = obj.state_t_pre(1);
            x = obj.state_t_pre(2);
            y = obj.state_t_pre(3);
            
            %% add new landmarks into the state prediction for time t+1
            for i = 1:num_landmark
                n = size(obj.state_t_pre,1);
                id = find(obj.idx2num == nums(i), 1);
                if isempty(id)     % new landmark 
                    % new measurements 
                    zx = centres(i,1);
                    zy = centres(i,2);
                    
                    % new landmark position
                    lx = x + cos(theta)*zx - sin(theta)*zy;
                    ly = y + sin(theta)*zx + cos(theta)*zy;
                                       
                    % update state prediction
                    obj.state_t_pre = [obj.state_t_pre; lx; ly];  % add onto state prediction vector
                    % update idx2num 
                    obj.idx2num = [obj.idx2num; nums(i)];    % add onto idx2num                    
                    % update covariance prediction 
                    tmp = zeros(n+2);
                    tmp(1:n,1:n) = obj.Cov_t_pre;     % (3+2n) state_t (3+2n)
                    
                    Gz = [cos(theta) -sin(theta);
                          sin(theta) cos(theta)];
                      
                    Gx = zeros(2,n);
                    Gx(1,1) = -sin(theta)*zx - cos(theta)*zy;
                    Gx(2,1) = cos(theta)*zx - sin(theta)*zy;
                    Gx(1,2) = 1;
                    Gx(2,3) = 1;
                    
                    Rz = [obj.siglm 0; 0 obj.siglm];
                    
                    tmp(end-1:end,end-1:end) = Gx*obj.Cov_t_pre*Gx' + Gz*Rz*Gz';
                    tmp(end-1:end,1:n) = Gx*obj.Cov_t_pre;
                    tmp(1:n,end-1:end) = obj.Cov_t_pre*Gx';
                    tmp(end-1:end,1:n) = Gx*obj.Cov_t_pre;
                    
                    obj.Cov_t_pre = tmp;
                end
            end
            
            %% update length of obj.state_t_pre
            n = size(obj.state_t_pre,1);
            
            %% C_t matrix 
            C_t = zeros(2*num_landmark,n);
            for i = 1:num_landmark
                lx = obj.state_t_pre(3+2*i-1,1);
                ly = obj.state_t_pre(3+2*i,1);
                
                % 3*3
                C_t(2*i-1:2*i,1:3) = [sin(theta)*(x-lx) - cos(theta)*(y-ly) -cos(theta) -sin(theta);
                                      cos(theta)*(x-lx) + sin(theta)*(y-ly) sin(theta) -cos(theta)];
                
                % 2n*2n
                id = find(obj.idx2num == nums(i), 1);
                C_t(2*i-1:2*i,3+id*2-1:3+id*2) = [cos(theta) sin(theta);
                                                  -sin(theta) cos(theta)];

%                 disp(C_t);
            end
            %% Z 
            % measurements estimation
            Z_pre = zeros(num_landmark*2,1);
            Z = zeros(num_landmark*2,1);
            for i = 1:num_landmark
                id = find(obj.idx2num == nums(i), 1);
                lx = obj.state_t_pre(3+2*id-1,1);
                ly = obj.state_t_pre(3+2*id,1);
                
                Z_pre(2*i-1:2*i,1) = [-cos(theta)*(x-lx)-sin(theta)*(y-ly);
                                      sin(theta)*(x-lx)-cos(theta)*(y-ly)];

                Z(2*i-1:2*i,1) = [centres(i,1);centres(i,2)];
            end
            
            
            %% Kt
            Qt = eye(2*num_landmark) .* obj.siglm;
            
            K_t = obj.Cov_t_pre*(C_t')*inv(C_t*obj.Cov_t_pre*C_t'+Qt);
            
            %% Covariance update
            obj.Cov_t = (eye(n) - K_t*C_t) * obj.Cov_t_pre;
            
           
            %% state_t
            obj.state_t = obj.state_t_pre - K_t * (Z_pre - Z);
        end
        
        function [robot, cov] = output_robot(obj)
            %% output robot state and cov
            robot = obj.state_t(1:3,1);   
            cov = obj.Cov_t(1:3,1:3);
        end
        
        function [num, position, cov] = output_landmarks(obj)
            %% output num, position and covariance
            n = size(obj.state_t,1);
            number = (n - 3) / 2;
            position = zeros(number,3);
            % num
            num = obj.idx2num;
            % position
            for i = 1:number
            position(i,1) = obj.idx2num(i,1);     
            position(i,2) = (obj.state_t(3+2*i-1,1));
            position(i,3) = (obj.state_t(3+2*i,1));
            end
            % cov
            cov = zeros(2*number,2);  
            
            for i = 1:number
                cov(2*i-1,:) = obj.Cov_t(3+i*2-1,3+i*2-1:3+i*2);
                cov(2*i,:) = obj.Cov_t(3+i*2,3+i*2-1:3+i*2);
            end
        end  
    end
end

