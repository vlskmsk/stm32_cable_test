%%

figure(1)
hold off
plot(time,log(:,1));
hold on
for i = 2:size(log,2);
    plot(time,log(:,i));
end
hold off

%%
tc = cast(log(:,1), 'uint16');
ts = cast(log(:,2), 'uint16');
ec = cast(log(:,3), 'uint16');
es = cast(log(:,4), 'uint16');

%%
plot(tc);
hold on
plot(ts);
plot(ec);
plot(es);
hold off

cos_lookup = zeros(4096,1);
sin_lookup = zeros(4096,1);

for i = 9998:11960
    cos_lookup(tc(i)+1) = ec(i);
    sin_lookup(ts(i)+1) = es(i);
end

%%
figure(1)
plot(cos_lookup);
hold on
plot(sin_lookup);
hold off

figure(2)
plot(tc);
hold on

plot(ts);
plot(ec);
plot(es);

plot(cos_lookup(tc+1));

hold off




