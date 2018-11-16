%% atan2 approximation test

theta_list = 0:.000001:2*pi;
num_methods = 2;
max_err = zeros(1,num_methods);
err = zeros(1,num_methods);
test_theta = zeros(1,num_methods);
for i = 1:length(theta_list)
    
    theta = theta_list(i);
    
    sinVal = sin(theta);
    cosVal = cos(theta);

    comp_v = atan2(sinVal,cosVal);
    
    test_theta(1) = atan2_fast(sinVal,cosVal);
    test_theta(2) = atan2_faster(sinVal,cosVal);
    
    for m = 1:num_methods
        err(m) = abs(comp_v - test_theta(m));
        if(err(m) > max_err(m))
            max_err(m) = err(m);
        end        
    end
    
end

%% full no trig alternative

e = 0.01;
theta_encoder = pi/3;
theta_offset = pi/4;

control_s = sin(theta_encoder+theta_offset);
control_c = cos(theta_encoder+theta_offset);

test_s = (sin(theta_encoder) + .01)*cos(theta_offset) + sin(theta_offset)*(cos(theta_encoder)-.01);
test_c = (cos(theta_encoder) + .011)*cos(theta_offset) - (sin(theta_encoder)+.001)*sin(theta_offset);

err_s = control_s - test_s;
err_c = control_c - test_c;

msg_buf = sprintf('err sin = %f, err_cos = %f\n', err_s, err_c);
disp(msg_buf);

%%

test_angle = 10*pi/4;
test_angle = atan2(sin(test_angle),cos(test_angle)); %% negate quadrants III and IV

th = atan2_fast(sin(test_angle),cos(test_angle))
res = atan2_fast(sin_fast(test_angle) , sin_fast(test_angle+pi/2))
