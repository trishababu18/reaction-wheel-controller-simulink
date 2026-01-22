function[wbidotstar, wbistar, angle] = trajectory_scalar(thetastar,J_C_P,hwmax,hwdotmax,safety,t)

%Calculate maximum allowable satellite angular acceleration
wbidotmax = safety*hwdotmax/max(max(J_C_P));

%Calculate maximum allowable satellite angular velocity
wbimax = safety*hwmax/max(max(J_C_P));

%Calculate time to torque at maximum until maximum angular velocity is
%reached
tmax = wbimax/wbidotmax;

%Calculate angle that can be traveled during maximum torque
anglemax = 1/2*wbidotmax*tmax^2;

% If the angle traveled during maximum torque is greater than half the
% total angle then the trajectry will be triangle not a trapezoid
if anglemax > thetastar/2

    %the time at the top of the triangle
    t1 = sqrt(thetastar/wbidotmax);

    %The angle traveled at t1 
    angle1 = thetastar/2;

    %the angular velocity at t1
    w1 = wbidotmax*t1;

    %Because we are a triangle there is not going to be any coasting period
    t2 = t1;
    angle2 = angle1;

    % The final time will be twice the time it took to get to the top 
    tf = t2 + t1;

else 

    %Calculate the angle traveled during the coast period 
    anglecoast = thetastar - 2*anglemax;

    %Calculate the time to coast
    tcoast = anglecoast/wbimax;
    
    %Torque for the maximum allowable amount of time
    t1 = tmax;

    %Calculate the angle traveled at that point
    angle1 = anglemax;

    %Calculate the angular velocity at that point 
    w1 = wbimax;

    %Calculate the time at the end of the coast 
    t2 = t1 + tcoast;

    %Calculate the angle traveled at that point
    angle2 = angle1 + w1*tcoast;

    %Calculate the final time 
    tf = t2 + t1;

end 

%After the final team hold the position
if t >= tf

    wbidotstar = 0;
    wbistar = 0;
    angle = thetastar;

    %Deceleration time period
elseif t>= t2

    %Torque at negative maximum 
    wbidotstar = -wbidotmax;

    %Agular velocity decrease linearly
    wbistar = w1 - wbidotmax*(t - t2);

    %Angle decreases parabolically
    angle = angle2 + w1*(t - t2) - 1/2*wbidotmax*(t - t2)^2;

    %Coasting time period 
elseif t >= t1

    %No acceleration 
    wbidotstar = 0;

    %Constant angular velocity 
    wbistar = w1;
   
    %Angle increases linearly
    angle = angle1 + w1*(t - t1);

else 

    %Accelerating time period 
    %Torque at maximum allowable
    wbidotstar = wbidotmax;

    %Angular velocity increases linearly
    wbistar = wbidotmax*t;

    %Angle increases parabolically
    angle = 1/2*wbidotstar*t^2;

end 

end 

