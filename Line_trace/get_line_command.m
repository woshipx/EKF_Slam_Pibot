function [lin_velocity, ang_velocity] = get_line_command(img)
% Settings
kP = 0.35;
kD = 0.23;
lin_velocity = 0.12;

bw_thresh = 0.25;

top_rows = 51:80;
bottom_rows = 1:30;

% Extract two segments of the line
top_image = img(end-top_rows,:,:);
bottom_image = img(end-bottom_rows,:,:);
top_gray = rgb2gray(top_image);
bottom_gray = rgb2gray(bottom_image);
top_threshed = ~imbinarize(top_gray, bw_thresh);
bottom_threshed = ~imbinarize(bottom_gray, bw_thresh);

% Check that the line segments were found
if sum(sum(top_threshed)) <= 0
    error("Top line segment could not be detected. Threshold used: %f.", bw_thresh);
end
if sum(sum(bottom_threshed)) <= 0
    error("Bottom line segment could not be detected. Threshold used: %f.", bw_thresh);
end

% Find the horizontal centres of the line segments and normalize to [-1, 1]
top_locations = repmat(1:size(top_threshed,2),size(top_threshed,1),1);
bottom_locations = repmat(1:size(bottom_threshed,2),size(bottom_threshed,1),1);
top_centre = sum(sum(top_threshed.*top_locations)) / sum(sum(top_threshed));
bottom_centre = sum(sum(bottom_threshed.*bottom_locations)) / sum(sum(bottom_threshed));
top_centre = (top_centre - size(img,1)/2) / (size(img,1)/2);
bottom_centre = (bottom_centre - size(img,1)/2) / (size(img,1)/2);

% Control the angular velocity to follow the line
ang_velocity = -kP*top_centre - kD*(top_centre - bottom_centre);


end