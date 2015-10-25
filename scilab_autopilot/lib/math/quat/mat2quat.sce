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

function [q_AB] = dcm2quat(dcm_AB)
    tmp1 = [1+dcm_AB(1,1)+dcm_AB(2,2)+dcm_AB(3,3)];
    tmp2 = [1+dcm_AB(1,1)-dcm_AB(2,2)-dcm_AB(3,3)];
    if (tmp1>tmp2) then
        q0 = 0.5 * sqrt(tmp1);
        den = 0.25 / q0;
        q1 = (dcm_AB(3,2)-dcm_AB(2,3)) * den;
        q2 = (dcm_AB(1,3)-dcm_AB(3,1)) * den;
        q3 = (dcm_AB(2,1)-dcm_AB(1,2)) * den;
    else
        q1 = 0.5 * sqrt(tmp2);
        den = 0.25 / q1;
        q2 = (dcm_AB(1,2)+dcm_AB(2,1)) * den;
        q3 = (dcm_AB(1,3)+dcm_AB(3,1)) * den;
        q0 = (dcm_AB(3,2)-dcm_AB(2,3)) * den;
    end
    q_AB = [q0;q1;q2;q3];
endfunction
