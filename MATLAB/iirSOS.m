function [y,w] = iirSOS(w,b0,b1,b2,gain,a1,a2, x)
    w(3) = w(2);
    w(2) = w(1);
	w(1) = x - a1*w(2) - a2*w(3);
    y = gain*(b0*w(1) + b1*w(2) + b2*w(3));   
end
