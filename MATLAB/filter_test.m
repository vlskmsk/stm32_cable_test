

SOS_test = SOS;
%% filter test

t = 0:.001:10;
sig = sin(2*pi*t*.1)+sin(2*pi*20*t);

sig_matlab_f = sosfilt(SOS_test,sig);

b = SOS_test(:,1:3);
a = SOS_test(:,4:6);
w = zeros(size(b,1),3);
custom_filt_sig = zeros(1,length(sig));
for sig_idx = 1:length(sig)

    i = 1;
    [filtered_val,w(i,:)] = iirSOS(w(i,:),b(i,1),b(i,2),b(i,3),1,a(i,2),a(i,3),sig(sig_idx));
    for i=2:size(b,1) 
        [filtered_val,w(i,:)] = iirSOS(w(i,:),b(i,1),b(i,2),b(i,3),1,a(i,2),a(i,3),filtered_val);
    end
    custom_filt_sig(sig_idx) = filtered_val;
end

figure(1)
plot(t*1000,sig);
% hold on
figure(2)
plot(t*1000,sig_matlab_f);
figure(3)
plot(t*1000,custom_filt_sig);
figure(4)
plot(t*1000,VarName1);
% hold off