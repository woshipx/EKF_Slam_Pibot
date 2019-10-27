function [B,flag] = Stop_mark(img)
% Settings
flag = 0;

bw_thresh = 0.3;

bottom_rows = 1;
% Extract two segments of the line
bottom_image = img(end-bottom_rows,:,:);
bottom_gray = rgb2gray(bottom_image);
bottom_threshed = ~imbinarize(bottom_gray, bw_thresh);
Top_image = img(end-60,:,:);
Top_gray = rgb2gray(Top_image);
Top_threshed = ~imbinarize(Top_gray, bw_thresh);

B = sum(sum(bottom_threshed));
T = sum(sum(Top_threshed));
if B > 60 && B - T > 40
        flag = 1;

end

end