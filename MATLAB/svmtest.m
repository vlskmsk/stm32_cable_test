%% 
t = 0:.001:3;
i_alpha = zeros(1,length(t));
i_beta = zeros(1,length(t));
tA = zeros(1,length(t));
tB = zeros(1,length(t));
tC = zeros(1,length(t));
vA = tA;
vB = tB;
vC = tC;
iA = zeros(1,length(t));
iB = zeros(1,length(t));
iC = zeros(1,length(t));
sector = zeros(1,length(t));

iaout = zeros(1,length(t));
ibout = zeros(1,length(t));
iqout = t;
idout = t;

theta = 3.14*4*t;
R = 1.3;
L = 35/1000000;
K = 0.00016;
mtr_elec = tf(1, [L, R]);

for i = 1:length(theta)
    [i_alpha(i), i_beta(i)] = inverse_park(sqrt(3)/2, 0, theta(i));
%     [tA(i), tB(i), tC(i), sector(i)] = svm_f(i_alpha(i), i_beta(i), 1000);
    [tA(i), tB(i), tC(i), sector(i)] = svm_func(i_alpha(i), i_beta(i), 1000);
%     [tA(i), tB(i), tC(i)] = inverse_clarke(i_alpha(i),i_beta(i));
    
    vA(i) = tA(i)*8.4/1000;
    vB(i) = tB(i)*8.4/1000;
    vC(i) = tC(i)*8.4/1000;

end


iA = lsim(mtr_elec, vA, t);
iB = lsim(mtr_elec, vB, t);
iC = lsim(mtr_elec, vC, t);

for i = 1:length(theta)
    [iaout(i), ibout(i)] = clarke_full(iA(i),iB(i),iC(i));
%     [iaout(i), ibout(i)] = clarke_full(iA(i),iB(i),iC(i));
    [iqout(i), idout(i)] = park(iaout(i),ibout(i),theta(i));
end

figure(1)
clf;
hold on
% plot(vA);
% plot(vB);
% plot(vC);
plot(iA);
plot(iB);
plot(iC);
plot(iqout);
plot(idout);
hold off
