clear all;
clc;
% Aruco Matlab
addpath("ARUCO_Detection_MATLAB")
addpath("ARUCO_Detection_MATLAB/include")
addpath("ARUCO_Detection_MATLAB/dictionary")
addpath("Line_trace")
addpath("Draw")
load("arucoDict.mat")
load("cameraParams.mat")

% --------------------------------------------
% connect
pb = PiBot('192.168.50.1');

% parameters
i_max = 350;

% save data
data = cell(5,i_max+1);
data1 = cell(5,i_max+1);
data2 = cell(3,i_max+1);

state = [0;0;0];        % integration kinematics [x;y;theta]
EKF = ekf_slam();
marker_length =0.07;
forward_v = 0;
angular_v = 0;
wheel_velocities = [0; 0];
% tic
time_last = 0;
dt = 0;
num_stop = 0;
figure;

for i = 0:i_max
 tic
 img = pb.getImage();

%% Kalman Prediction
    EKF.input_velocity(dt, forward_v, angular_v);
    [marker_nums, landmark_centres, marker_corners] = Aruco_detector(img, cameraParams, arucoDict);
%     [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);
    EKF.input_measurements(landmark_centres, marker_nums);
    
    % output states
    [robot_state, robot_cov]= EKF.output_robot();   % robot_state = [theta;x;y]
    [landmark_num, landmark_position, landmark_cov] = EKF.output_landmarks();

 %% Line following
[forward_v, angular_v] = get_line_command(img);
 [B,flag] = Stop_mark(img);
if flag == 1
        pb.setLEDArray(1023);
%     for j = 1:4
%         forward_v = 0;
%         angular_v = pi/2;
%         [wheel_velocities] = reparam_controlL(forward_v, angular_v);
%         [forward_v,angular_v] = velocity_calibration(wheel_velocities);
%         pb.setVelocity(wheel_velocities,1);
%     end

    num_stop = num_stop + 1;
end
pb.setLEDArray(0);
 %% Set velocity 
%     if B < 5 && i > 170 
%      forward_v = 0;
%      angular_v = 0; 
%     [wheel_velocities] = reparam_controlL(forward_v, angular_v);
%     [forward_v,angular_v] = velocity_calibration(wheel_velocities);
%     pb.setVelocity(wheel_velocities,10);          
%     else
    if num_stop == 5
     forward_v = 0;
     angular_v = 0; 
    [wheel_velocities] = reparam_controlL(forward_v, angular_v);
    [forward_v,angular_v] = velocity_calibration(wheel_velocities);
    pb.setVelocity(wheel_velocities,10);
   
    state = integrate_kinematics(state,dt,forward_v,angular_v); 
    else   
    [wheel_velocities] = reparam_controlL(forward_v, angular_v);
    [forward_v,angular_v] = velocity_calibration(wheel_velocities);

    disp(wheel_velocities');

    pb.setVelocity(wheel_velocities);
%     dt = toc;   
    state = integrate_kinematics(state,dt,forward_v,angular_v); 
%     end
    end
%% draw
        figure(1)
%     subplot(2,1,1);
% %         draw_image_1(img);
%         plot(state(1),state(2),'.');
%         hold on
%         plot(robot_state(2),robot_state(3),'*');
%      subplot(2,1,1);
%     draw_image(img,marker_corners);
% 
%     subplot(2,1,2);
draw_path(state,robot_state,landmark_num,landmark_position,landmark_cov,flag);

%% save data
    dt = toc;
    if dt > 0.9
        dt = 0.9;    %control start time when robot begin
    end
    data(:,i+1) = {i,dt,[forward_v angular_v],wheel_velocities,img};
    data1(:,i+1) = {i,dt,robot_state(1),robot_state(2),robot_state(3)};
    data2(:,i+1) = {flag,B,0};

end

pb.stop();
