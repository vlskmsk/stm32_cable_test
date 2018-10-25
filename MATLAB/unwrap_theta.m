function [theta_ret,prev_theta_ret] = unwrap_theta(theta,prev_theta)
   	dif = mod( ( mod(prev_theta,2*pi) - theta + pi),  2*pi);
    if(dif < 0)
		dif = dif + 2*pi;
    end
	dif = dif - pi;
	theta_ret = prev_theta - dif;
	prev_theta_ret = theta_ret;
end