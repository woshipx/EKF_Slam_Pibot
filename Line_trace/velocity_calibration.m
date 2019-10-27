function [forward_v,angular_v] = velocity_calibration(wheel_velocities)
%Calibrate the wheel_velocity outputs to real forward_v and angular_v
%   Input: wheel_velocities
%   Output: [forward_v,angular_v]

L = floor(wheel_velocities(1)/5)*5;
R = floor(wheel_velocities(2)/5)*5;

%% stright
if (L == R)
    switch L
        case 40
            forward_v = 0.21;       
        case 35
            forward_v = 0.19436346;
        case 30
            forward_v = 0.167189133;
        case 25
            forward_v = 0.141643059;
        case 20
            forward_v = 0.113960114;
        case 15
            forward_v = 0.085763293;
        case 10
            forward_v = 0.056862606;
        case 5
            forward_v = 0.028376844;
        case 0
            forward_v = 0;
    end
    angular_v = 0;
    
    return;
end

%% turn: forward

% L 
switch L
     case 45
        L_forward = 0.23;       
    case 40
        L_forward = 0.21;    
    case 35
        L_forward = 0.19436346;
    case 30
        L_forward = 0.167189133;
    case 25
        L_forward = 0.141643059;
    case 20
        L_forward = 0.113960114;
    case 15
        L_forward = 0.085763293;
    case 10
        L_forward = 0.056862606;
    case 5
        L_forward = 0.028376844;
    otherwise
        error('Left wheel speed error!');
end

% R
switch R
    case 45
            R_forward = 0.23;
    case 40
            R_forward = 0.21;     
    case 35
        R_forward = 0.19436346;
    case 30
        R_forward = 0.167189133;
    case 25
        R_forward = 0.141643059;
    case 20
        R_forward = 0.113960114;
    case 15
        R_forward = 0.085763293;
    case 10
        R_forward = 0.056862606;
    case 5
        R_forward = 0.028376844;
    otherwise
        error('Right wheel speed error!');
end
forward_v = (L_forward + R_forward) / 2;

%% turn: angualr

V = [num2str(L) ' ' num2str(R)];
switch V
    % right
    case '40 35'
        angular_v = 0;
    case '40 25'
        angular_v = 1.02*-0.188783475;  
    case '40 20'
        angular_v = 1.02*-0.380683793; 
    case '40 15'
        angular_v = 1.02*-0.576968411; 
    case '40 10'
        angular_v = 1.02*-0.756782415; 

    case '35 30'
        angular_v = 1.02*-0.188783475;
    case '35 25'
        angular_v = 1.02*-0.380683793;
    case '35 20'
        angular_v = 1.02*-0.576968411;
    case '35 15'
        angular_v = 1.02*-0.756782415;
    case '35 10'
        angular_v = 1.02*-0.955981134;
        
    case '30 25'
        angular_v = 1.02*-0.188712599;
    case '30 20'
        angular_v = 1.02*-0.378961761;
    case '30 15'
        angular_v = 1.02*-0.578029991;
    case '30 10'
        angular_v = 1.02*-0.762753991;
        
    case '25 20'
        angular_v = 1.02*-0.18888279;
    case '25 15'
        angular_v = 1.02*-0.381492775;
    case '25 10'
        angular_v = 1.02*-0.575778786;
        
    case '20 15'
        angular_v = 1.02*-0.18963815;
    case '20 10'
        angular_v = 1.02*-0.382363365;
        
    case '15 10'
        angular_v = 1.02*-0.190054023;
    case '15 5'
        angular_v = 1.02*-0.385589813;
        
    case '10 5'
        angular_v = 1.02*-0.191765176;
        
    % left
    case '35 40'
        angular_v = 0;
    case '30 40'
        angular_v = 0;
    case '25 40'
        angular_v =1.02*0.188783475;  
    case '20 40'
        angular_v = 1.02*0.380683793; 
    case '15 40'
        angular_v = 1.02*0.576968411; 
    case '10 40'
        angular_v = 1.02*0.756782415; 
    case '30 35'
        angular_v = 1.02*0.188996421;
    case '25 35'
        angular_v = 1.02*0.389232523;
    case '20 35'
        angular_v = 1.02*0.576968411;
    case '15 35'
        angular_v = 1.02*0.772602029;
    case '10 35'
        angular_v = 1.02*0.961099197;
        
    case '25 30'
        angular_v = 1.02*0.196672228;
    case '20 30'
        angular_v = 1.02*0.387611721;
    case '15 30'
        angular_v = 1.02*0.579362471;
    case '10 30'
        angular_v = 1.02*0.771179626;
        
    case '20 25'
        angular_v = 1.02*0.195646458;
    case '15 25'
        angular_v = 1.02*0.3863604;
    case '10 25'
        angular_v = 1.02*0.574594056;
        
    case '15 20'
        angular_v = 1.02*0.194586126;
    case '10 20'
        angular_v = 1.02*0.387054579;
        
    case '10 15'
        angular_v = 1.02*0.193507422;
    case '5 15'
        angular_v = 1.02*0.391353846;
        
    case '5 10'
        angular_v = 1.02*0.193686375;
        
    otherwise 
        error('angualr velocity error!')
end

end

