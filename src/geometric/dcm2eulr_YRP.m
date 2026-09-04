function eul_vect = dcm2eulr_YRP(DCMbn)
  
  phi = asin(-DCMbn(2,3));
  theta = atan2(DCMbn(1,3),DCMbn(3,3));
  psi = atan2(DCMbn(2,1),DCMbn(2,2));

  eul_vect = [phi theta psi]';
