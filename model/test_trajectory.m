t = 0:0.1:30;

angle = zeros(1,length(t));
wbistar = zeros(1,length(t));
wbidotstar = zeros(1,length(t));

thetastar = 10*pi/180;
J_C_P = [0.00721530225864069	-0.00195174445995786	0.000385309197883204;
-0.00195174445995785	0.0384087162651311	-9.77549765690555e-05;
0.000385309197883204	-9.77549765690553e-05	0.0399284479393799];

hwmax = 0.0150;
hwdotmax = 0.0040;
safety = 0.5;

for i = 1:length(t)
    [wbidotstar(i),wbistar(i),angle(i)] = trajectory_scalar(thetastar,J_C_P,hwmax,hwdotmax,safety,t(i));
end 

figure 
plot(t,angle);
title('Profiled angle');

figure 
plot(t,wbistar);
title('Profiled angular velocity');

figure 
plot(t,wbidotstar);
title('Profiled angular acceleration');
