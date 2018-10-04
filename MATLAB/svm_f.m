function [tA,tB,tC,sector] = svm_f(alpha,beta, pwm_half_period)
    SQRT_3 = sqrt(3);
    ONE_BY_SQRT_3 = 1/(SQRT_3);
    TWO_BY_SQRT_3 = 2*ONE_BY_SQRT_3;
    if(beta >= 0.0)	%quadrant 1 or two
	
        if(alpha >= 0.0)	%quadrant 1
            if(beta <= alpha*SQRT_3)	%sector 1
				sector = 1;
			else							%sector 2
				sector = 2;
            end
        else				%quadrant 2
		
            if(beta <= alpha*-SQRT_3)	%sector 2
				sector = 3;
			else							%sector 3
				sector = 2;
            end
        end
    
	else			%quadrant 3 or 4
        if(alpha >= 0.0)	%quadrant 4		
                if(beta < -SQRT_3*alpha)
                    sector = 5;
                else
                    sector = 6;
                end
            else				%quadrant 3

                if(beta < alpha*SQRT_3)
                    sector = 5;
                else
                    sector = 4;
                end
        end
    end
    
    switch (sector)

        case 1

            t1 = ((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
            t2 = ((TWO_BY_SQRT_3 * beta) * pwm_half_period);
            tA = (pwm_half_period - t1 - t2) / 2;
            tB = tA + t1;
            tC = tB + t2;

        case 2

            t2 = ((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
            t3 = ((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
            tB = (pwm_half_period - t2 - t3) / 2;
            tA = tB + t3;
            tC = tA + t2;

        case 3

            t3 = ((TWO_BY_SQRT_3 * beta) * pwm_half_period);
            t4 = ((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
            tB = (pwm_half_period - t3 - t4) / 2;
            tC = tB + t3;
            tA = tC + t4;

        case 4

            t4 = ((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
            t5 = ((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
            tC = (pwm_half_period - t4 - t5) / 2;
            tB = tC + t5;
            tA = tB + t4;

        case 5

            t5 = ((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
            t6 = ((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
            tC = (pwm_half_period - t5 - t6) / 2;
            tA = tC + t5;
            tB = tA + t6;

        case 6

            t6 = ((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
            t1 = ((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
            tA = (pwm_half_period - t6 - t1) / 2;
            tC = tA + t1;
            tB = tC + t6;
    end
end