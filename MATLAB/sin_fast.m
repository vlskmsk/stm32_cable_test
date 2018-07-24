function y = sin_fast(theta)
    ONE_BY_THREE_FACTORIAL  =   0.16666666666;
    ONE_BY_FIVE_FACTORIAL   =   0.00833333333;
    HALF_PI                 =   1.57079632679;
    PI                      =   3.14159265359;
    THREE_BY_TWO_PI         =   4.71238898038;
    TWO_PI                  =   6.28318530718;
	is_neg = 0;
	if(theta > HALF_PI && theta <= PI)	% if positive and in quadrant II, put in quadrant I (same)
		theta = PI - theta;
    elseif (theta >= PI && theta < THREE_BY_TWO_PI)  % if positive and in quadrant III (possible for cosine)
        is_neg = 1;
        theta = theta - PI;
    elseif (theta > THREE_BY_TWO_PI && theta < TWO_PI)  % if positive and in quadrant IV (edge case of cosine, rare but possible)
        theta = theta - TWO_PI;
    elseif (theta < -HALF_PI && theta >= -PI ) % if negative and in quadrant III,
		is_neg = 1;
		theta = PI + theta;
    end
    
	theta_2 = theta*theta;
	theta_3 = theta_2*theta;
	theta_5 = theta_3*theta_2;
	res = theta-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL;
	if(is_neg == 1)
		y = -res;
	else
		y = res;
    end
end