////////////////////////////////////////////////////////////////////////////////
// Controller tuning
//
// m*s^2*X = Kp*(Xg-X) + Kd*s*(Xg-X) + Ki/s*(Xg-X) 
// (m*s^2 + Kp + Kd*s + Ki/s)*X = (Kp + Kd*s + Ki/s)*Xg 
// (m*s^3 + Kd*s^2 + Kp*s + Ki)*X = (Kd*s^2 + Kp*s + Ki)*Xg 
//
//  X         Kd/Ki*s^2 + Kp/Ki*s + 1
// --- = ----------------------------------
//  Xg   m/Ki*s^3 + Kd/Ki*s^2 + Kp/Ki*s + 1
//
//
// Tuning strategy:
// - tune the PD to get the adequate time response
//      m*s^2*X = Kp*(Xg-X) + Kd*s*(Xg-X) 
//       X         Kd/Kp*s + 1               (s/w1*s + 1)
//      --- = ---------------------- = -------------------------
//       Xg   m/Kp*s^2 + Kd/Kp*s + 1   (s/w0)^2 + 2*ksi/w0*s + 1
// - maximize the integral term while keeping the module / phase / gain margins
//

s = poly(0, 's');

A = [
    0 1 ;
    0 0 ;
];
B = [
    0 ;
    1/dynamics.mass ;
];
C = eye(2,2);
D = [
    0 ; 
    0 ;
];
G = syslin('c',A,B,C,D);

navigation.w0 = [
    2*%pi/10 ;
    2*%pi/10 ;
    2*%pi/1  ;
];

navigation.ksi = [
    0.7 ;
    0.7 ;
    0.7 ;
];

navigation.Kp = navigation.w0.^2 * dynamics.mass ;
navigation.Kd = 2*navigation.ksi ./ navigation.w0 .* navigation.Kp ;
navigation.Ki = navigation.w0/10;
navigation.maxDistFrc = dynamics.massErr * max(abs(modulator.frcMaxPos_B), abs(modulator.frcMaxNeg_B)) + 1*ones(3,1);

A = 0;
B = [1 0];
C = navigation.Ki;
D = [
    navigation.Kp navigation.Kd ;
];
K = syslin('c',A,B,C,D);

nicholschart
KG = syslin('c',K(1,:)*G);
black(KG)
KG = syslin('c',K(2,:)*G);
black(KG)
KG = syslin('c',K(3,:)*G);
black(KG)



////////////////////////////////////////////////////////////////////////////////
// Direct thrust guidance

navigation.guid.maxThrust = [0 0 32145/1200 * 2^16];
