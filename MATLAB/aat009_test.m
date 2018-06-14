%% motor position test

t = 0:.01:2;
A = 1;
offset = 0;
s = 2*A*sin(t*2*pi)+offset-1;
c = 2*A*cos(t*2*pi)+offset-1;

figure(1)
plot(t,s);
hold on
plot(t,c);
hold off

angle = atan2(s,c);
mag = sqrt(s.*s + c.*c);
x = sin(angle).*mag;
y = cos(angle).*mag;
figure(2)
plot(x,y);
