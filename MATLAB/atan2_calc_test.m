%% atan2 approximation test

theta_list = 0:.000001:2*pi;

max_err = 0;
for i = 1:length(theta_list)
    
    theta = theta_list(i);
    
    sinVal = sin(theta);
    cosVal = cos(theta);

    comp_v = atan2(sinVal,cosVal);


    abs_s = sinVal;
    if(abs_s < 0)
        abs_s = -abs_s;
    end
    abs_c = cosVal;
    if(abs_c < 0)
        abs_c = -abs_c;
    end
    min_v = abs_c;
    max_v = abs_s;
    if(abs_s < abs_c)
        min_v = abs_s;
        max_v = abs_c;
    end

    a = min_v/max_v;
    sv = a*a;
    r = ((-0.0464964749 * sv + 0.15931422)*sv- 0.327622764) * sv * a + a;
    if(abs_s > abs_c)
        r = 1.57079637 -r;
    end
    if(cosVal < 0)
        r = 3.14159274 - r;
    end
    if(sinVal < 0)
        r = -r;
    end

    err = comp_v - r;
    if(err > max_err)
        max_err = err;
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
