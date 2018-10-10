%% 
clear all
theta = 5.47;
[alpha,beta] = inverse_park(.3, 0.0, theta);
[tA,tB,tC,sector] = svm_f(alpha, beta, 1000);  

[Va,Vb,Vc] = inverse_clarke(alpha,beta);

Va_svm = (tA-500)/1000;
Vb_svm = (tB-500)/1000;
Vc_svm = (tC-500)/1000;

%%
clear all
theta_elec = 0:.01:6*pi;
len = length(theta_elec);
tA = zeros(1,len);
tB = zeros(1,len);
tC = zeros(1,len);
sector = zeros(1,len);
for i = 1:len
    [alpha,beta] = inverse_park(.3, 0.0, theta_elec(i));
    [tA(i),tB(i),tC(i),sector(i)] = svm_f(alpha,beta,1000);
end

figure(1);
plot(tA);
hold on
plot(tB);
plot(tC);
plot(sector*100);
hold off;

