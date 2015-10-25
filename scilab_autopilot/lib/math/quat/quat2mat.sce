// Get the dcm encoded by the quaternion
//
// The computed dcm is the matrix such that
//              ____
// q_AB * v_B * q_AB = dcm_AB * v_B
//
// INTPUT
// - q_AB: input quaternion
//
// OUTPUT
// - dcm_AB: dcm B --> A
//
// USAGE
// [dcm_AB] = quat2mat(q_AB);
//
// HISTORY
// 09/02/2015: T. Pareaud - Creation

function [dcm_AB] = quat2dcm(q_AB)
    q0 = q_AB(1,1);
    q1 = q_AB(2,1);
    q2 = q_AB(3,1);
    q3 = q_AB(4,1);
    dcm_AB = [ 
        (q0^2 + q1^2 - q2^2 - q3^2) 2*(q1*q2 - q0*q3) 2*(q0*q2 + q1*q3) ;
        2*(q1*q2 + q0*q3) (q0^2 - q1^2 + q2^2 - q3^2) 2*(q2*q3 - q0*q1) ;
        2*(q1*q3 - q0*q2) 2*(q0*q2 + q1*q3) (q0^2 - q1^2 - q2^2 + q3^2) ;
    ];
endfunction
