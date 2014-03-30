function [res] = vect_crossProd(v1, v2)
    res = [ v1(2,:).*v2(3,:) - v1(3,:).*v2(2,:) ; v1(3,:).*v2(1,:) - v1(1,:).*v2(3,:) ; v1(1,:).*v2(2,:) - v1(2,:).*v2(1,:) ];
endfunction
