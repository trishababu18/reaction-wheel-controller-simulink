function [wbidotstar_P, wbistar_P, qstar_PI] = trajectory(qstar_BI, q0_BI, q_PB, J_C_P, hwmax, hwdotmax,safety,t)

% wbidotstar_P = zeros(3,1);
% wbistar_P = zeros(3,1);
% qstar_PI = qstar_BI;
 
%Project intial and desired quaternion to principal frame 
qstar_PI = qX(q_PB,qstar_BI);
qstar_PI = qUnit(qstar_PI);

q0_PI = qX(q_PB,q0_BI);
q0_PI = qUnit(q0_PI);

%Calculate the error quaternion between desired and initial
qe = qX(qstar_PI,qT(q0_PI));
qe = qUnit(qe);

%Convert the quaternion to an axis and angle 
[e, thetastar] = q2e(qe);

[wbidotstar, wbistar, angle] = trajectory_scalar(thetastar,J_C_P,hwmax,hwdotmax,safety,t);

%Project acceleration and velocity to the P frame 
wbidotstar_P = wbidotstar*e;
wbistar_P = wbistar*e;

%Create the desired quaternion 
qrot = e2q(e,angle);
qstar_PI = qX(qrot,q0_PI);
qstar_PI = qUnit(qstar_PI);

end 

