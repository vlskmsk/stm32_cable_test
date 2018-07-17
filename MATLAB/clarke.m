function [i_alpha, i_beta] = clarke(i_a, i_b)
    i_alpha = i_a;
	i_beta = (1/sqrt(3))*i_a + (2/sqrt(3))*i_b;
end