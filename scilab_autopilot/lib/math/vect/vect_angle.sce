function ang = vect_angle(v1,v2)
    angSin = vect_norm(vect_crossProd(v1, v2));
    angCos = vect_dotProd(v1, v2);
    ang = atan(angSin, angCos);
endfunction
