// Motor position in body frame [m]
motors.pos_B = [
    0.1470  0.1470 -0.1450 -0.1450 ;
   -0.2325  0.2325  0.1450 -0.1450 ;
    0.0000  0.0000  0.0000  0.0000 ;
];

// Force direction [-]
motors.forceDir_B = [
    0.000  0.000  0.000  0.000 ;
    0.000  0.000  0.000  0.000 ;
   -1.000 -1.000 -1.000 -1.000 ;
];

// Lift force [N]
motors.forceLift = [
    6.867  6.867  6.867  6.867 ;
];

// Drag torque around force dir [Nm]
motors.torqueDrag = [
    1.400 -1.400  1.400 -1.400 ;
];
