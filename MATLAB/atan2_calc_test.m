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
