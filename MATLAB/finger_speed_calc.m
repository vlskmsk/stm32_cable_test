%% finger speed calculation

%constants
vishan_ratio = 21.3;
worm_ratio = 20;
finger_angle_degrees = 75;
acceleration_time_ms = 10;

%calc
motor_speed_Hz = 50000/60;
bone_Hz = motor_speed_Hz/vishan_ratio/worm_ratio;
time_to_close = (1/bone_Hz)*finger_angle_degrees/360 + acceleration_time_ms/1000;
disp(time_to_close);