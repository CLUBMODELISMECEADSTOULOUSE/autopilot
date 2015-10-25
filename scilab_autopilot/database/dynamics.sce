m = 1.140;
g = 9.81;

h=1.37;
dur = 36;
N = 15;
T = dur / N;
w = 2*%pi/T;
Jz = m*g*h / w^2 - m*h^2;

h=1.47;
dur = 27;
N = 11;
T = dur / N;
w = 2*%pi/T;
Jy = m*g*h / w^2 - m*h^2;

h=1.50;
dur = 25;
N = 10;
T = dur / N;
w = 2*%pi/T;
Jx = m*g*h / w^2 - m*h^2;

// Mass of the quadcopter [Kg]
dynamics.mass = 1.6;

// Inertia of the quadcopter [Kg.m^2]
dynamics.inertia = [
    Jx 0.000 0.000 ;
    0.000 Jy 0.000 ;
    0.000 0.000 Jz ;
];

// Center of mass [m]
dynamics.com_B = [
    0.000 ;
    0.000 ;
    0.000 ;
];

// Mass relatve error [%]
dynamics.massErr = 0.10;

// CoM absolute error [m]
dynamics.com_B_err = [
    0.020 ;
    0.020 ;
    0.000 ;
];
