%% second attempt at lookup table using atan2 to linearize both functions

tc_f = log(:,1);
ts_f = log(:,2);
ec_f = log(:,3);
es_f = log(:,4);
mid_tc = (max(tc_f)+min(tc_f))/2;
mid_ts = (max(ts_f)+min(ts_f))/2;
mid_ec = (max(ec_f)+min(ec_f))/2;
mid_es = (max(es_f)+min(es_f))/2;

theta_t = atan2(ts_f-mid_ts,tc_f-mid_tc);
theta_e = atan2(es_f-mid_es,ec_f-mid_ec);

theta_t_i = cast(((theta_t+pi)*651),'uint16');
theta_e_i = cast(((theta_e+pi)*651),'uint16');

%% TEST THE MAP

theta_lookup = (lookup_table( theta_t_i + 1 )*0.001536098310292) - pi;

figure(1)
% plot((theta_t_i/651) - pi );
plot(theta_e)
hold on
plot(theta_lookup);
plot(theta_e-theta_lookup);
hold off
%%  CALCULATE THE MAP
theta_map = zeros(4096,1);
for i = 1:length(theta_e_i)
    theta_map(theta_t_i(i)+1) = theta_e_i(i);
end

%%  everything else is wrong. gotta build the map on atan2, not sin and cos
% t = 0:.01:2;
% test1 = 2*sin(t*2*pi);
% test2 = sin(t*2*pi);
% plot(t,test1);
% hold on
% plot(t,test2)
% hold off
% 
% 
% 
% 
% 
% %%
% 
% figure(1)
% hold off
% plot(time,log(:,1));
% hold on
% for i = 2:size(log,2);
%     plot(time,log(:,i));
% end
% hold off
% 
% %%
% tc = cast(log(:,1), 'uint16');
% ts = cast(log(:,2), 'uint16');
% ec = cast(log(:,3), 'uint16');
% es = cast(log(:,4), 'uint16');
% 
% 
% %%
% 
% tc_shift = padarray(tc,0);
% 
% plot(tc_shift);
% hold on
% % plot(ts);
% plot(ec);
% % plot(es);
% hold off
% 
% cos_lookup = zeros(4096,1);
% sin_lookup = zeros(4096,1);
% 
% for i = 9486:9767
%     cos_lookup(tc_shift(i)+1) = ec(i);
%     sin_lookup(ts(i)+1) = es(i);
% end
% % for i = 1:4096
% %     if(cos_lookup(i) == 0)
% %         cos_lookup(i) = ec(i);
% %     end
% %     if (sin_lookup(i) == 0)
% %         sin_lookup(i) = es(i);
% %     end
% % end
% 
% 
% 
% figure(1)
% plot(cos_lookup);
% hold on
% plot(sin_lookup);
% hold off
% 
% figure(2)
% % plot(tc);
% hold on
% 
% % plot(ts);
% % plot(ec);
% % plot(es);
% 
% plot(cos_lookup(tc+1));
% 
% hold off
% 
% 
% 
% 
