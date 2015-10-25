
cd ../database
exec fsw.sce
exec dynamics.sce
exec motors.sce

cd ../tuning
exec modulator.sce
exec attitude.sce
exec navigation.sce

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Generate the NRD
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Dynamics

nrd.dynamics.mass = dynamics.mass;
nrd.dynamics.inertia = dynamics.inertia;

////////////////////////////////////////////////////////////////////////////////
// Modulator

// Influence Matrix
nrd.modulator.inflMat = round(modulator.inflMat * (2^fsw.SCALE_INFLUENCE_MATRIX * 2^fsw.SCALE_TORSOR) / (fsw.PWM_MAX - fsw.PWM_MIN));

// CAPA
nrd.modulator.frcMaxPos_B = round(modulator.frcMaxPos_B * 2^fsw.SCALE_TORSOR) ;
nrd.modulator.frcMaxNeg_B = round(modulator.frcMaxNeg_B * 2^fsw.SCALE_TORSOR) ;
nrd.modulator.trqMaxPos_B = round(modulator.trqMaxPos_B * 2^fsw.SCALE_TORSOR) ;
nrd.modulator.trqMaxNeg_B = round(modulator.trqMaxNeg_B * 2^fsw.SCALE_TORSOR) ;

// LUT
nrd.modulator.lutTrq = round(modulator.lutTrq * (fsw.PWM_MAX - fsw.PWM_MIN)) ;
nrd.modulator.lutFrc = round(modulator.lutFrc * (fsw.PWM_MAX - fsw.PWM_MIN)) ;

////////////////////////////////////////////////////////////////////////////////
// Attitude Controller

nrd.attitude.Kp = attitude.Kp;
nrd.attitude.Kd = attitude.Kd;
nrd.attitude.Ki = attitude.Ki;
nrd.attitude.maxI = attitude.maxDistTrq;
nrd.attitude.maxCosAngOverTwoErr = cos(attitude.maxCtrlErr/2) ;
nrd.attitude.maxSinAngOverTwoErr = sin(attitude.maxCtrlErr/2) ;

F = ss2tf(attitude.filter.F(1,1));
nrd.attitude.filterX.num = coeff(numer(F));
nrd.attitude.filterX.den = coeff(denom(F));
F = ss2tf(attitude.filter.F(2,2));
nrd.attitude.filterY.num = coeff(numer(F));
nrd.attitude.filterY.den = coeff(denom(F));
F = ss2tf(attitude.filter.F(3,3));
nrd.attitude.filterZ.num = coeff(numer(F));
nrd.attitude.filterZ.den = coeff(denom(F));


////////////////////////////////////////////////////////////////////////////////
// Navigation Controller

nrd.navigation.Kp = navigation.Kp ;
nrd.navigation.Kd = navigation.Kd ;
nrd.navigation.Ki = navigation.Ki ;
nrd.navigation.maxI = navigation.maxDistFrc ;

