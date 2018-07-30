s = serial('com7');
s.BaudRate = 921600;
s.Terminator = 'CR/LF';
fopen(s);

% figure;
% H = uicontrol('Style','text','String', 'close figure to exit',...
%                 'Position',[20,20,100,50]);

numReadVals = 4;
log = zeros(10000,numReadVals);
time = zeros(length(log),1);
log_idx = 1;

lookup_table = zeros(4096,1);
num_zeros = length(lookup_table);
% while(ishandle(H))
tic;
while(num_zeros > 10)
    serLog = fread(s,numReadVals,'int16');
    tc_f = serLog(1);
    ts_f = serLog(2);
    ec_f = serLog(3);
    es_f = serLog(4);
    theta_t = atan2(ts_f-mid_ts,tc_f-mid_tc);
    theta_e = atan2(es_f-mid_es,ec_f-mid_ec);
    theta_t_i = cast(((theta_t+pi)*651),'uint16');
    theta_e_i = cast(((theta_e+pi)*651),'uint16');
    if(lookup_table(theta_t_i+1) == 0)
        lookup_table(theta_t_i+1) = theta_e_i;
        num_zeros = num_zeros - 1;
    end
%     if(toc > .1)
%         disp(num_zeros);
%         tic;
%     end
end

fclose(s);
delete(s);
clear s;

plot(lookup_table);