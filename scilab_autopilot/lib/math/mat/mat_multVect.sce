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

function [vOut] = mat_multVect(m, vIn)
    nRow = size(m,1);
    nCol = size(m,2);
    N = max(size(m,3),size(vIn,2));
    vOut = zeros(nRow,N);
    for iRow = 1:nRow
        for iCol = 1:nCol
            vOut(iRow,:) = vOut(iRow,:) + matrix(m(iRow,iCol,:),1,size(m,3)).*vIn(iRow,:);
        end
    end
endfunction
