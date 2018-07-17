function [i_alpha, i_beta] = clarke_full(i_a, i_b, i_c)
	
    ib_m_ic = (i_b-i_c);
	i_alpha = (2/3)*i_a - (1/3)*(i_b + i_c);
	i_beta = (2/sqrt(3))*ib_m_ic;
end