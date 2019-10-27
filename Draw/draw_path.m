function draw_path(state,robot_state,landmark_num,landmark_position,landmark_cov,flag)


persistent robot_state_last;
if isempty(robot_state_last)
   robot_state_last = [0;0;0];
end

persistent N_landmark;
persistent P_landmark;
persistent Ellipse_Position;


% Plot the reference path 
if  robot_state_last == [0;0;0]
    draw_map();
end

%Integration Kinematics
hold on;
plot( state(1), state(2),'.');
hold off;

%% Robot_trajection
hold on;
plot([robot_state_last(2) robot_state(2)],[robot_state_last(3) robot_state(3)],'b','linewidth',2);
robot_state_last = robot_state;
hold off;

%% landmarks
hold on;
if(~isempty(P_landmark))
    delete(P_landmark);
end
if(~isempty(N_landmark))
    delete(N_landmark);
end
if(~isempty(landmark_num))
    P_landmark = plot(landmark_position(:,2),landmark_position(:,3),'r.');
    N_landmark = text(landmark_position(:,2),landmark_position(:,3),num2str(landmark_num));
end
hold off;

% Plot landmarks error ellipse 
hold on;
if (~isempty(Ellipse_Position))
    delete(Ellipse_Position);
end
if(~isempty(landmark_num))
    n = size(landmark_num,1);
    ellipse_last = [];
    for i = 1:n
        cov = landmark_cov(2*i-1:2*i,:);
        ellipse_last =  [ellipse_last; draw_error_ellipse(landmark_position(i,2:3),cov)];
    end
    Ellipse_Position = ellipse_last;
end
hold off;

%% Stop Flag 
hold on;
if flag == 1
    plot(robot_state(2),robot_state(3),'o');
end
% if flag == 1
%     plot(state(1),state(2),'o');
% end
hold off;

drawnow;

end