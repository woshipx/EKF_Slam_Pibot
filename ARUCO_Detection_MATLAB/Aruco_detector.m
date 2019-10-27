function [marker_nums, landmark_centres, marker_corners] = Aruco_detector(img, cameraParams, arucoDict)
%ARUCO_DETECTOR only return information for marker 0 to 19
%   Detailed explanation goes here

marker_length = 0.07;  % mark length

[marker_nums, landmark_centres, marker_corners] = detectArucoPoses(img, marker_length, cameraParams, arucoDict);

% take num 0 - 19 only
valid_marker = find(marker_nums<=19 & marker_nums>=0);
marker_nums = marker_nums(valid_marker);
landmark_centres = landmark_centres(valid_marker,:);
marker_corners = marker_corners(valid_marker,:,:);



end

