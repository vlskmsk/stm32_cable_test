% function [i_alpha, i_beta] = clarke_full(i_a, i_b, i_c)
% 	
%     ib_m_ic = (i_b-i_c);
% 	i_alpha = (2/3)*i_a - (1/3)*(i_b + i_c);
% 	i_beta = (2/sqrt(3))*ib_m_ic;
% end
function [i_alpha, i_beta] = clarke_full(i_a, i_b, i_c)
    ONE_BY_SQRT_3 = 1/sqrt(3);
	i_alpha = -i_b - i_c;
	i_beta = ONE_BY_SQRT_3 * (i_b - i_c);
end

