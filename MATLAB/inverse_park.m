function [i_alpha,i_beta] = inverse_park(iq, id, theta)
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    i_alpha = id*cos_theta - iq*sin_theta;
	i_beta = id*sin_theta + iq*cos_theta;
end