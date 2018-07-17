%%
a = .5;
theta = 3*30*pi/180;
ia = a*sin(theta);
ib = a*sin(theta+2*pi/3);
ic = a*sin(theta+4*pi/3);
% ia = .5;
% ib = -.5;
% ic = 0;
fprintf('before:(%f, %f, %f)\n', ia,ib,ic);
% [i_alpha, i_beta] = clarke_full(ia,ib,ic);
[i_alpha, i_beta] = clarke(ia,ib);
[ia,i_b,ic] = inverse_clarke(i_alpha,i_beta);
fprintf('after:(%f, %f, %f)\n', ia,ib,ic);

%%
[ialpha,ibeta] = inverse_park(0.0, 0.2, 0);
[ia,ib,ic] = inverse_clarke(ialpha,ibeta);
fprintf('(%f, %f, %f)\n', ia,ib,ic);


%%

[ialpha,ibeta] = clarke(i_b,i_c);
theta = atan2(ialpha,ibeta);
for i = 1:length(theta)
    [iq(i),id(i)] = park(ialpha(i),ibeta(i),theta(i));
end
figure(2)
plot(ialpha)
hold on
plot(ibeta)
hold off

figure(3)
plot(iq);
hold on
plot(id);
hold off
%%

close all
figure(2)
plot(i_b);
hold on
plot(i_c);
plot(1899*sin(theta));
plot(1858*sin(theta+2*pi/3));
hold off
