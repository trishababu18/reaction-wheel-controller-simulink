%% Reaction Wheel Control Assignment 
% Name: Trisha Babu 

%% Preliminaries
% This clears all variables and sets the format to disp more digits.
clearvars
close all
clc
format long

%% Control Preferences
% ctrlpref

%% Addpath to Required Folders
addpath('../Attitude representations')
addpath('../Attitude Kinematics')
addpath('../Attitude Dynamics')

%%
load qBus.mat;

%% Load mass properties
mass_properties;

%%
% Reaction Wheel properties 
wn = 2*pi*10; % Reaction wheel natural frequency
zeta = sqrt(2)/2; % Reaction wheel damping ratio 
hwmax = 0.015; % Nms
hwdotmax = 0.004; % Nm
safety = 0.5;

%%
% Initial reaction wheel angular momentum 
hw0_B = [0;0;0];

%% Parameters for Design and Simulation
% Control System Time Delay
dt_delay = 0.01; % seconds

%% Initial Satellite Attitude
% Body axes aligned with inertial axes 
q0_BI.s = 1;
q0_BI.v = [0;0;0];

%% Desired satellite attitude 
% Create a desired attitude quaternion by roattating through 90 degrees by x
% axis 
e = [1;2;3]; 
e = e/norm(e);
qstar_BI = e2q(e,180*pi/180);

%%
% The intial attitude in the simulation can be specified by converting into
% inertial DCM
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

%% Initial satellite angular velocity 

wbi0_B = [0;0;0];%rad/s

%% Inner loop Controller design
% Design proportional control gains such that all three axes have identical responses.  Use the following parameters and satisfy the following requirements:
% Following Control Design Requirements
% Gain Margin >= 10 dB
% Phase Margin >= 60 degrees
% Rise Time <= 0.025 seconds
% Overshoot <= 1%

%% Transfer Function Variable
s = tf('s');

%% Plant Model for Each Axis

G1 = 1/(J_C_P(1,1) * s);
display(G1);
G2 = 1/(J_C_P(2,2) * s);
display(G2);
G3 = 1/(J_C_P(3,3) * s);
display(G3);

%% 
% Define reaction wheel transfer function 
Gw = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

figure
bode(Gw);
title('Reaction wheel Closed loop bode plot');

figure
step(Gw);
title('Reaction wheel Closed loop step response');

% Configure control system designer
% [num, den] = pade(dt_delay,8);
% C_pade8 = tf(num,den);
% 
% config = sisoinit(1);
% config.G.value = G1;
% config.C.value = C_pade8*Gw;
% % config.OL1.View = {'bode'};
% % config.CL2.View = {'bode'};
% controlSystemDesigner(config);
%%
% Select the gain crossover frequency to use for all three plants 
w_crossover = 2*pi*2.5; %rad/s
display(w_crossover);

%% Design a proportional gain for each of the three principle axes that meet the design requirements above. Choose the gains such that each of the three axes have identical responses.
[num,den] = pade(dt_delay,8);
C_pade8 = tf(num,den);

Kd1 = 1 / bode(G1*Gw, w_crossover);
display(Kd1);
Kd2 = 1 / bode(G2*Gw, w_crossover);
display(Kd2);
Kd3 = 1 / bode(G3*Gw, w_crossover);
display(Kd3);

Kd = diag([Kd1; Kd2; Kd3]);

%% Controller with delay by cascading the proportional gain

C1 = Kd1 * C_pade8;
display(C1);
C2 = Kd2 * C_pade8;
display(C2);
C3 = Kd3 * C_pade8;
display(C3);

%% Open Loop Bode Plot of controlled plant 
figure
bode(C1*G1*Gw, C2*G2*Gw, C3*G3*Gw, {1, 1000}, '--');
title('Inner Open Loop Bode Plots');
legend('Axis 1', 'Axis 2', 'Axis 3');

[Gm1,Pm1] = margin(C1*G1*Gw);
[Gm2,Pm2] = margin(C2*G2*Gw);
[Gm3,Pm3] = margin(C3*G3*Gw);

display(mag2db(Gm1), 'Inner Gain margin 1 (dB)');
display( Pm1, 'Inner Phase margin 1 (degrees)');

display(mag2db(Gm2), 'Inner Gain margin 2 (dB)');
display( Pm2, 'Inner Phase margin 2 (degrees)');

display(mag2db(Gm3), 'Inner Gain margin 3 (dB)');
display( Pm3, 'Inner Phase margin 3 (degrees)');

%% Closed Loop Transfer Functions
CLTF1 = feedback(C1 * G1 * Gw, 1);
CLTF2 = feedback(C2 * G2 * Gw, 1);
CLTF3 = feedback(C3 * G3 * Gw, 1);

%% Closed Loop Bode Plots
figure
bode(CLTF1, CLTF2, CLTF3, {1, 1000}, '--');
title('Inner closed Loop Bode Plots');
legend('Axis 1', 'Axis 2', 'Axis 3');

%% Closed Loop 3dB Bandwidth
BW1 = bandwidth(CLTF1)/(2*pi);
BW2 = bandwidth(CLTF2)/(2*pi);
BW3 = bandwidth(CLTF3)/(2*pi);

display(BW1, 'Inner Closed Loop 3 dB Bandwidth Axis 1 (Hz)');
display(BW2, 'Inner Closed Loop 3 dB Bandwidth Axis 2 (Hz)');
display(BW3, 'Inner Closed Loop 3 dB Bandwidth Axis 3 (Hz)');

%% Step Response Information
stepinfo1 = stepinfo(CLTF1);
stepinfo2 = stepinfo(CLTF2);
stepinfo3 = stepinfo(CLTF3);
display(stepinfo1);
display(stepinfo2);
display(stepinfo3);

%% Outer loop proportional design
%%
% Configure control system designer to start in correct loop configuration

% config = sisoinit(6);
% config.G1.value = G1;
% config.C1.value = 1;
% config.C2.value = Kd1*C_pade8*Gw;
% config.G2.value = 1/s;
% config.OL1.View = {'bode'};
% config.OL2.View = {};
% controlSystemDesigner(config);

Go = 1/s;

Kp = 1/bode(CLTF1*Go,2*pi*1);
OLTF = Kp*CLTF1*Go;

figure;
bode(OLTF,{0.001,1000});
title('Outer Proportional open loop bode plot');
[Gm, Pm]= margin(OLTF);

display(mag2db(Gm),'Outer proportional gain margin');
display(Pm,'Outer proportional gain margin(deg)');

% %% Step info of the proportional closed loop transfer function 
% stepinfo_outer = stepinfo(CLTF);
% display(stepinfo_outer);

%% Closed Loop Bode plot of controlled plant 

CLTF = feedback(OLTF,1);
figure;
bode(CLTF,{1,1000});
title('Outer proportional closed loop bode plot');
display(bandwidth(CLTF)/2*pi),'Outer Closed loop 3dB bandwidth(Hz)';

stepinfo_outer = stepinfo(CLTF);
display(stepinfo_outer);

figure
step(CLTF)
title('Closed Loop Step Response');

%% Outer loop design with integrator and lead 

z = 2*pi*0.0001;
p = 2*pi*100;
Co = (s + z)/(s*(s+p));
K = 1/bode(Co*CLTF1*Go,2*pi*1);
Co = K*Co;

OLTF = Co*CLTF1*Go;

figure;
bode(OLTF,{0.001,1000});
title('Outer open loop bode plot');
[Gm,Pm] = margin(OLTF);
display(mag2db(Gm), 'Outer gain margin(dB)');
display(Pm, 'Outer phase margin(degrees)');

%% Disturbance transfer functions 
C1 = C_pade8*Kd1*Gw;
C2 = C_pade8*Kd2*Gw;
C3 = C_pade8*Kd3*Gw;

%The disturbance transfer function is given by:
disturbance1 = G1*Go/(1+C1*G1+Co*C1*G1*Go);
disturbance2 = G2*Go/(1+C2*G2+Co*C2*G2*Go);
disturbance3 = G3*Go/(1+C3*G3+Co*C3*G3*Go);

figure
bode(disturbance1,disturbance2,disturbance3,{1e-6,1000});
title('Disturbance transfer functions');

figure
step(disturbance1,disturbance2,disturbance3);
title('Step disturbance rejection');

%% Simulink 
sim('reaction_wheel_1.slx');


