%%
s = serial('com7');
s.BaudRate = 921600;
s.Terminator = 'CR/LF';
fopen(s);

figure;
H = uicontrol();

numReadVals = 1;
bufSize = 500;
buf1 = nan(1,bufSize);
% buf2 = nan(1,bufSize);

fwrite(s,'P');
tic;
min_v = [4096,4096];
max_v = [0,0];
eq_v = zeros(200,1);     %this might not trigger properly. adding some tolerance might be necessary
eq_v_idx = 1;
while(ishandle(H))
    serLog = fread(s,numReadVals,'int16');
    
    
    
%     for i = 1:2
%         if(serLog(i) < min_v(i))
%             min_v(i) = serLog(i);
% %             disp(serLog);
%         end
%         if(serLog(i) > max_v(i))
%             max_v(i) = serLog(i);
% %             disp(serLog);
%         end
%     end
%     
%     abs_diff = abs(serLog(1)-serLog(2));
%     if(abs_diff < 1)
%         disp(serLog);
%         eq_v(eq_v_idx) = serLog(1);
%         eq_v_idx = eq_v_idx+1;
%         if(eq_v_idx>length(eq_v))
%             eq_v_idx = 1;           %just fucking wrap that shit i guess
%         end
%     end
%   
    
%     buf1 = [buf1(2:end), serLog(1)];
%     buf2 = [buf2(2:end), serLog(2)];
%     hold off
%     plot(buf1(1,:));
%     hold on
%     plot(buf2(1,:));
%     hold off

    disp(serLog);
    
    pause(0.00001);
end
fwrite(s,'P');

fclose(s);
delete(s);
clear s;
%%

mid_sin = (max_v(1)+min_v(1))/2;
mid_cos = (max_v(2)+min_v(2))/2;
Ad = max_v(1)-min_v(1);
Av = Ad*3.3/4096;

top_45 = 0;
bottom_45 = 0;
for i = 1 : length(eq_v)
    if(eq_v(i) ~= 0)
        if(eq_v(i) > mid_cos)
            top_45(i) = eq_v(i);
        end
        if(eq_v(i) < mid_cos)
            bottom_45(i) = eq_v(i);
        end
    end
end
avg_top = mean(top_45);
avg_bottom = mean(bottom_45);
est_max = avg_top/(sin(pi/4));
est_min = avg_bottom/(sin(pi/4));
est_mid = (avg_top+avg_bottom)/2;

%%
theta = 5*pi/4;
ang_deg = atan2(sin(theta),cos(theta))*180/pi;
if(ang_deg < 0)
    ang_deg = ang_deg+360
end




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
log = zeros(5000,numReadVals);
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

%% more post

iA = log(:,1);
iB = log(:,2);
iC = log(:,3);

plot(time, iA);
hold on
plot(time, iB);
plot(time, iC);
% plot(time, iA+iB+iC);
hold off


