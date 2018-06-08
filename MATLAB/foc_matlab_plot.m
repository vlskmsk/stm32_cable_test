
Fs = 9259;
t = 0:1/Fs:2;
sig = 100*sin(t*23500*2*pi)+sin(t*500*2*pi);
w = zeros(3,3);
filtsig_iirsos = zeros(1,length(t));
filt = 0;
for i = 1:length(t)
    section = 1;
    [filt,w(section,:)] = iirSOS(w(section,:),SOS(section,1),SOS(section,2),SOS(section,3),G(section),SOS(section,5),SOS(section,6),sig(i));
%     section = 2;
%     [filt,w(section,:)] = iirSOS(w(section,:),SOS(section,1),SOS(section,2),SOS(section,3),G(section),SOS(section,5),SOS(section,6),filt);
%     section = 3;
%     [filt,w(section,:)] = iirSOS(w(section,:),SOS(section,1),SOS(section,2),SOS(section,3),G(section),SOS(section,5),SOS(section,6),filt);
    filtsig_iirsos(i) = filt;
 end

filtsig = sosfilt(SOS,sig);
figure(1)
plot(sig(1:300))
hold on
plot(filtsig(1:300))
hold off
figure(2)
plot(sig(1:300));
hold on
plot(filtsig_iirsos(1:300));
hold off

%% 
t = (t_ms-t_ms(1))/1000;
avg_st=zeros(length(t),1);
for i=2:length(t)
   avg_st(i) = t(i)-t(i-1); 
end
P =mean(avg_st);
Fs = 1/P;
% I_mA_filt = sosfilt(SOS,I_mA);

% figure(1)
% plot(t, I_mA_filt);
figure(2)
plot(t, I_mA);
%%
s = serial('com7');
s.BaudRate = 921600;
s.Terminator = 'CR/LF';
fopen(s);

figure;
H = uicontrol('Style','text','String', 'close figure to exit',...
                'Position',[20,20,100,50]);
         
while(ishandle(H))
    serLog = fscanf(s,'%s');
    disp(serLog);
    pause(0.0000001);
end


fclose(s);
delete(s);
clear s;