function draw_image(img,marker_corners)

imshow(img);
hold on;
nums = size(marker_corners,1);
for j = 1:nums
    x = [marker_corners(j,:,1) marker_corners(j,1,1)];
    y = [marker_corners(j,:,2) marker_corners(j,1,2)];
    plot(x,y,'y','LineWidth',2);
end

hold off;

drawnow;

end
