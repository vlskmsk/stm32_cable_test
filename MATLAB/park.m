function [iq,id] = park(i_alpha,i_beta,theta)
    cos_theta = cos(theta);
    sin_theta = sin(theta);
	id = i_alpha*cos_theta+i_beta*sin_theta;
	iq = i_beta*cos_theta - i_alpha*sin_theta;
end

