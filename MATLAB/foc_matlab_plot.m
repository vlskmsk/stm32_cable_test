
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
t = (tms-tms(1))/1000;
avg_st=zeros(length(t),1);
for i=2:length(t)
   avg_st(i) = t(i)-t(i-1); 
end
P =mean(avg_st);
Fs = 1/P;
% curr_filt = sosfilt(SOS,curr);

% figure(1)
% plot(t, curr_filt);
figure(2)
plot(t, curr);


%%
Fs = 10000;
t = 0:1/Fs:.333;
I_emul = sin(t*2*pi*30);
for i = 1:length(t)
    if(I_emul(i) > 0)
        I_emul(i) = 0;
    end
end
plot(t,I_emul)
%%
s = serial('com7');
s.BaudRate = 921600;
s.Terminator = 'CR/LF';
fopen(s);

% figure;
% H = uicontrol('Style','text','String', 'close figure to exit',...
%                 'Position',[20,20,100,50]);

log = zeros(1000,1);
time = zeros(length(log),1);
log_idx = 1;

% while(ishandle(H))
tic;
while(1)
    serLog = fread(s,1,'int16');
    disp(serLog);
    log(log_idx) = serLog;
    time(log_idx)= toc;
    log_idx = log_idx + 1;
    if(log_idx > length(log))
%         log_idx = 1;            %wrap around when overflow
        break;  %quit when buffer is full
    end
    pause(0.0000000001);
end

fclose(s);
delete(s);
clear s;

plot(time,log);
