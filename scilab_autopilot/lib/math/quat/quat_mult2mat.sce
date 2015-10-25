// Multiplication multiplication as a matrix
//
// Hamilton multiplication as a matrix
// [m1] = quat_mult2mat(q1)
// q1*q2 = m1*q2
//
// INTPUT
// - q: a quaternion
//
// OUTPUT
// - m: equivalent matrix
//
// USAGE
// [m] = quat_mult2mat(q);
//
// HISTORY
// 09/02/2015: T. Pareaud - Creation

function [m] = quat_mult2mat(q)
    s=q(1,1);
    v1=q(2,1);
    v2=q(3,1);
    v3=q(4,1);
    m = [
         s -v1 -v2 -v3 ;
        v1   s -v3  v2 ;
        v2  v3   s -v1 ;
        v3 -v2  v1   s ;
    ];
endfunction
