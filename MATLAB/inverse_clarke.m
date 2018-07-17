function [i_a,i_b,i_c] = inverse_clarke(i_alpha,i_beta)
    sqrt_3_by_2 = sqrt(3)*.5;
	i_a = i_alpha;
	i_b = -.5*i_alpha + sqrt_3_by_2*i_beta;
	i_c = -.5*i_alpha - sqrt_3_by_2*i_beta;
end
