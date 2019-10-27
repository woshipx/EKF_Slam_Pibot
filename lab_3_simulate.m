clear all;
clc;

% Aruco
addpath("ARUCO_Detection_MATLAB")
addpath("ARUCO_Detection_MATLAB/include")
addpath("ARUCO_Detection_MATLAB/dictionary")
addpath("Line_trace")
addpath("Draw")
load("arucoDict.mat")
load("cameraParams.mat")

% load 
   load Dataset/data_test.mat
% parameters
i_max = 400;
marker_length = 0.085;
% run
state = [0;0;0];        % integration kinematics [x;y;theta]
EKF = ekf_slam_simu();

forward_v = 0;
angular_v = 0;
data1=cell(3,i_max+1);
num_stop = 0;
figure;
for i = 0:i_max
    img = data{5,i+1};   

    dt = data{2,i+1};
    
    %% Kalman Prediction
    EKF.input_velocity(dt, forward_v, angular_v);
    
    % measurement 
    [marker_nums, landmark_centres, marker_corners] = Aruco_detector(img, cameraParams, arucoDict);
    EKF.input_measurements(landmark_centres, marker_nums);
    
    %% output states
    [robot_state, robot_cov]= EKF.output_robot();   % robot_state = [theta;x;y]
    [landmark_num, landmark_position, landmark_cov] = EKF.output_landmarks();   
  
%% integration kinematics

   

    forward_v = data{3,i+1}(1);
    angular_v = data{3,i+1}(2);
    [wheel_velocities] = reparam_controlL(forward_v, angular_v);
    [forward_v,angular_v] = velocity_calibration(wheel_velocities);
    state = integrate_kinematics(state,dt,forward_v,angular_v);  % use new data
%     disp(robot_state); 
 [B,flag] = Stop_mark(img);
 if flag == 1
     num_stop = num_stop + 1;
 end
%% draw
    figure(1)
    subplot(2,1,1);
%     draw_image(img,marker_corners);
        draw_image(img, marker_corners);
    subplot(2,1,2);
    
    draw_path(state,robot_state,landmark_num,landmark_position,landmark_cov,flag)

data1(:,i+1) = {flag,B,0};
end