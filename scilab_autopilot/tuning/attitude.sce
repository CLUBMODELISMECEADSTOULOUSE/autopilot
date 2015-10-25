////////////////////////////////////////////////////////////////////////////////
// Controller tuning
//
// J*s^2*X = Kp*(Xg-X) + Kd*s*(Xg-X) + Ki/s*(Xg-X) 
// (J*s^2 + Kp + Kd*s + Ki/s)*X = (Kp + Kd*s + Ki/s)*Xg 
// (J*s^3 + Kd*s^2 + Kp*s + Ki)*X = (Kd*s^2 + Kp*s + Ki)*Xg 
//
//  X         Kd/Ki*s^2 + Kp/Ki*s + 1
// --- = ----------------------------------
//  Xg   J/Ki*s^3 + Kd/Ki*s^2 + Kp/Ki*s + 1
//
//
// Tuning strategy:
// - tune the PD to get the adequate time response
//      J*s^2*X = Kp*(Xg-X) + Kd*s*(Xg-X) 
//       X         Kd/Kp*s + 1               (s/w1*s + 1)
//      --- = ---------------------- = -------------------------
//       Xg   J/Kp*s^2 + Kd/Kp*s + 1   (s/w0)^2 + 2*ksi/w0*s + 1
// - maximize the integral term while keeping the module / phase / gain margins
//
// Version vitesse
// J*s*X = Kd*(Xg-X) 
// (J*s+Kd)*X = Kd*Xg
//  X       1
// -- = --------
// Xg   J/Kd*s+1

s = poly(0, 's');
tau=0.10;
delay = tf2ss(syslin('c',(1+1/2*(-tau*s)+1/10*(-tau*s)^2+1/120*(-tau*s)^3)/(1-1/2*(-tau*s)+1/10*(-tau*s)^2-1/120*(-tau*s)^3))) * eye(3,3);


A = [
    zeros(3,3) eye(3,3) ;
    zeros(3,3) zeros(3,3) ;
];
B = [
    zeros(3,3) ;
    inv(dynamics.inertia) ;
];
C = eye(6,6) ;
D = zeros(6,3) ;
G = syslin('c',A,B,C,D) * delay;
Gd = dscr(G,0.01);
Gd.dt = 0.01;

attitude.w0 = [
    2*%pi/3.0 ;
    2*%pi/3.0 ;
    2*%pi/5.0  ;
];

attitude.ksi = [
    1.0 ;
    1.0 ;
    0.7 ;
];

attitude.Kp = attitude.w0.^2 .* diag(dynamics.inertia) ;
attitude.Kd = 2*attitude.ksi ./ attitude.w0 .* attitude.Kp ;
attitude.Ki = attitude.w0/10;

attitude.filter.w0 = attitude.w0 * 0.5 ;
attitude.filter.w1 = attitude.w0 * 2 ;

//attitude.filter.F = [
//    cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(1))+1)/((s/attitude.filter.w1(1))^2+2*0.7*s/attitude.filter.w1(1)+1))),0.01) 0 0 ;
//    0 cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(2))+1)/((s/attitude.filter.w1(2))^2+2*0.7*s/attitude.filter.w1(2)+1))),0.01) 0 ;
//    0 0 cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(3))+1)/((s/attitude.filter.w1(3))^2+2*0.7*s/attitude.filter.w1(3)+1))),0.01) ;
//];
attitude.filter.F = [
    cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(1))+1)/((s/attitude.filter.w1(1))+1))),0.01) 0 0 ;
    0 cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(2))+1)/((s/attitude.filter.w1(2))+1))),0.01) 0 ;
    0 0 cls2dls(tf2ss(syslin('c',((s/attitude.filter.w0(3))+1)/((s/attitude.filter.w1(3))+1))),0.01) ;
];
attitude.filter.F.dt = 0.01;

A = eye(3,3);
B = 0*[sysdiag(attitude.Ki(1),attitude.Ki(2),attitude.Ki(3))*0.01 zeros(3,3)];
C = eye(3,3);
D = [
    sysdiag(attitude.Kp(1), attitude.Kp(2), attitude.Kp(3)) sysdiag(attitude.Kd(1), attitude.Kd(2), attitude.Kd(3)) ;
];
K = syslin(0.01,A,B,C,D);

w0=2*%pi/0.1;
Deriv = syslin(0.01,(1-(1/s))/0.01)
Fd=ss2tf(cls2dls(tf2ss(syslin('c',1/((s/w0)+1))),0.01));
h = ss2tf(tf2ss(Fd)*tf2ss(Deriv));
H = [eye(3,3), zeros(3,3) ; sysdiag(h,h,h), zeros(3,3)];

KG = attitude.filter.F * K*G;
//KG = K*H*G;
figure; 
nicholschart
black([KG(1,1);KG(2,2);KG(3,3)])
nicholschart
black([KG(1,1);KG(2,2);KG(3,3)])
figure; 
bode([KG(1,1);KG(2,2);KG(3,3)])


// Max disturbance for integral term saturation
attitude.maxDistTrq = abs(vect_crossProd(dynamics.com_B_err,max(abs(modulator.frcMaxPos_B),abs(modulator.frcMaxNeg_B)))) + 0.1 * ones(3,1) ;

// Max attitude control error under which small angle approximation is performed
attitude.maxCtrlErr = 45 * %pi / 180 ;

//  X         Kd/Ki*s^2 + Kp/Ki*s + 1
// --- = ----------------------------------
//  Xg   J/Ki*s^3 + Kd/Ki*s^2 + Kp/Ki*s + 1

