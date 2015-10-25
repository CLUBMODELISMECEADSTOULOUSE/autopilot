////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Modulator
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Influence Matrix

modulator.inflMat = [
    vect_scalProd(motors.forceLift, motors.forceDir_B) ;
    vect_crossProd(motors.pos_B, vect_scalProd(motors.forceLift, motors.forceDir_B)) + vect_scalProd(motors.torqueDrag, motors.forceDir_B) ;
];

Nmotor = size(modulator.inflMat,2);

////////////////////////////////////////////////////////////////////////////////
// Force domain

modulator.frcMaxPos_B = zeros(3,1);
modulator.frcMaxNeg_B = zeros(3,1);
modulator.lutFrc = zeros(Nmotor,3,2);

lb = zeros(Nmotor,1);
ub = ones(Nmotor,1);

for iAxis=1:3
    [xopt,fopt,exitflag] = karmarkar([],[],-modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
    modulator.frcMaxPos_B(iAxis) = -fopt;
    
    [xopt,fopt,exitflag] = karmarkar([],[], modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
    modulator.frcMaxNeg_B(iAxis) =  fopt;

    Aeq = modulator.inflMat([1:(iAxis-1) (iAxis+1):6],:);
    if (~ isempty(kernel(Aeq)))
        [xopt,fopt,exitflag] = karmarkar(Aeq,zeros(5,1),-modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
        if (exitflag == 1 & abs(fopt) > 0.1)
            modulator.lutFrc(:,iAxis,2) = -xopt/fopt;
        end
        [xopt,fopt,exitflag] = karmarkar(Aeq,zeros(5,1), modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
        if (exitflag == 1 & abs(fopt) > 0.1)
            modulator.lutFrc(:,iAxis,1) = -xopt/fopt;
        end
    end
end

////////////////////////////////////////////////////////////////////////////////
// Torque domain

modulator.trqMaxPos_B = zeros(3,1);
modulator.trqMaxNeg_B = zeros(3,1);
modulator.lutTrq = zeros(Nmotor,3,2);

lb = zeros(Nmotor,1);
ub = ones(Nmotor,1);

for iAxis=4:6
    [xopt,fopt,exitflag] = karmarkar([],[],-modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
    modulator.trqMaxPos_B(iAxis-3) = -fopt;
    
    [xopt,fopt,exitflag] = karmarkar([],[], modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
    modulator.trqMaxNeg_B(iAxis-3) =  fopt;

    Aeq = modulator.inflMat([4:(iAxis-1) (iAxis+1):6],:);
    if (~ isempty(kernel(Aeq)))
        [xopt,fopt,exitflag] = karmarkar(Aeq,zeros(2,1),-modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
        if (exitflag == 1 & abs(fopt) > 0.1)
            modulator.lutTrq(:,iAxis-3,2) = -xopt/fopt;
        end
        [xopt,fopt,exitflag] = karmarkar(Aeq,zeros(2,1), modulator.inflMat(iAxis,:)',[],[],[],[],[],[],[],lb,ub);
        if (exitflag == 1 & abs(fopt) > 0.1)
            modulator.lutTrq(:,iAxis-3,1) = -xopt/fopt;
        end
    end
end
