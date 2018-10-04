%% 
theta = 30*pi/180;
beta = .3*sin(theta);
alpha = .3*cos(theta);
[tA,tB,tC] = svm_f(alpha, beta, 1000);  

[Va,Vb,Vc] = inverse_clarke(alpha,beta);

Va_svm = (tA-500)/1000;
Vb_svm = (tB-500)/1000;
Vc_svm = (tC-500)/1000;

%%
theta = 0:.01:6*pi;
beta = .3*sin(theta);
alpha = .3*cos(theta);
[tA,tB,tC] = svm_f(alpha,beta,1000);
[vA,vB,vC] = inverse_clarke(alpha,beta);

vA = (vA*1000)+500;
vB = (vB*1000)+500;
vC = (vC*1000)+500;

figure(1);
plot(theta,tA);
hold on
plot(theta,tB);
plot(theta,tC);
hold off
figure(2);
plot(theta, vA);
hold on 
plot(theta, vB);
plot(theta, vC);
hold off;

