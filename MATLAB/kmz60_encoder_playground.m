%%
% Workspace for debugging all the issues that arise from having an encoder
% which returns twice the mechanical angle instead of the mechanical angle
% itsef

clear all
close all

%%
true_theta_m = pi/2+pi/4:.005:4*pi;
len = length(true_theta_m);

zero_line = zeros(1,length(true_theta_m));
% first, model encoder signal
theta_m = mod( (true_theta_m + pi), 2*pi )-pi;  % take the true angle, and wrap it

offset = 0;
sin_enc = sin(2*(true_theta_m+offset));  
cos_enc = cos(2*(true_theta_m+offset));  %these are the actual voltage signals 
theta_enc = atan2(sin_enc,cos_enc);

theta_elec = zeros(1,length(theta_enc));
prev_theta = 0;
for i = 1:len
    [theta_elec(i),prev_theta] = unwrap_theta(theta_enc(i),prev_theta);
    theta_elec(i) = theta_elec(i)*.5;
    theta_elec(i) = mod((theta_elec(i)+pi),2*pi)-pi;
end

clf
figure(1)
% plot(true_theta_m/pi);
hold on
plot(theta_m/pi);
plot(theta_elec/pi);
% plot(zero_line);
hold off

%%
figure(2)
plot(sin(theta_m))
hold on
% plot(cos(theta_m))
plot(sin(theta_enc));
% plot(cos(theta_enc));
hold off

