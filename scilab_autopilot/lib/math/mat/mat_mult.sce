// Multiplication of a matrix and a vector
//
// INTPUT
// - m: the matrix
// - vIN: the input vector
//
// OUTPUT
// - vOut: the output vector
//
// USAGE
// vOut = mat_multVect(m, vIn);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [m3] = mat_mult(m1, m2)
    nRow = size(m1,1);
    nCol1 = size(m1,2);
    nCol2 = size(m2,2);
    N = max(size(m1,3),size(m2,3));
    m3 = zeros(nRow,nCol2,N);
    for iRow = 1:nRow
        for iCol2 = 1:nCol2
            for iCol1 = 1:nCol1
                m3(iRow,iCol2,:) = m3(iRow,iCol2,:) + m1(iRow,iCol1,:).*m2(iCol1,iCol2,:);
            end
        end
    end
endfunction
