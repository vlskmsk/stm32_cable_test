%%
clear all
close all
%%
theta_elec = 0:.01:6*pi;
len = length(theta_elec);

tA = zeros(1,len);
tB = zeros(1,len);
tC = zeros(1,len);
sector = zeros(1,len);

for i = 1:len
    [alpha,beta] = inverse_park(.3, 0.0, theta_elec(i));
    [tA(i),tB(i),tC(i),sector(i)] = svm_sinusoidal(alpha,beta,1000);
end

tA = tA + 1000;
tB = tB + 500;

figure(1);
plot(tA);
hold on
plot(tB);
plot(tC);
% plot(sector*100);
hold off;

