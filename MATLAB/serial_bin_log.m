%%
s = serial('com7');
s.BaudRate = 921600;
s.Terminator = 'CR/LF';
fopen(s);

% figure;
% H = uicontrol('Style','text','String', 'close figure to exit',...
%                 'Position',[20,20,100,50]);

numReadVals = 3;
valScale = 1;
log = zeros(1000,numReadVals);
time = zeros(length(log),1);
log_idx = 1;

% while(ishandle(H))
tic;
while(1)
    serLog = fread(s,numReadVals,'int32');
%     disp(serLog);
    log(log_idx,:) = serLog;
    time(log_idx)= toc;
    log_idx = log_idx + 1;
    if(log_idx > length(log))
%         log_idx = 1;            %wrap around when overflow
        break;  %quit when buffer is full
    end
%     pause(0.01);
end

fclose(s);
delete(s);
clear s;

hold off
plot(time,log(:,1)*valScale);
hold on
for i = 2:size(log,2);
    plot(time,log(:,i)*valScale);
end
hold off

%% fft

avg_st=zeros(length(time),1);
for i=2:length(time)
   avg_st(i) = time(i)-time(i-1); 
end
P =mean(avg_st);
Fs = 1/P;

L = length(log);
X = fft(log)/L;
PSD = 2*abs(X(1:L/2+1));
f = Fs/2*linspace(0,1,L/2+1);
plot(f,PSD);

%% filtering
SOS_G = SOS;
for i = 1:size(SOS,1)
    for j = 1:3
        SOS_G(i,j) = SOS(i,j)*G(i);
    end    
end

filtsig = sosfilt(SOS_G,log);
plot(log)
hold on
plot(filtsig);
hold off