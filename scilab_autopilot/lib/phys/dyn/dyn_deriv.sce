function [dX] = dyn_deriv(X, wrench_O, mass, I)
	rate_B = dyn_getRate(X);
	Q_IB = dyn_getAtt(X);
	pos_I = dyn_getPos(X);
	vel_I = dyn_getVel(X);
    
    K = 0.1;
    quatCorr = K * (1 - quat_norm(Q_IB)) * Q_IB;

	H = I * rate_B;
	
	T_B = screw_getMoment(wrench_O) - vect_crossProd(rate_B, H) ;
	F_B = screw_getResultant(wrench_O);
	F_I = quat_rotate(Q_IB, F_B);
	
	// Compute derivative rate
	dRate = inv(I) * T_B;

	// Compute derivative velocity
	dVel = F_I / mass;
	dPos = vel_I;
	
	// Compute derivative attitude
    qRate = quat_new(0,rate_B);
    dQ = 1/2 .* quat_mult(Q_IB, qRate);
	
    dQ = dQ + quatCorr;
	
	// Compute derivative position
	dX = dyn_new(dPos, dVel, dQ, dRate);
endfunction
