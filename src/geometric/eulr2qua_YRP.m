function qua_vec = eulr2qua_YRP(eul_vect)

  phi = eul_vect(1); theta = eul_vect(2); psi = eul_vect(3);
  
  cthe2 = cos(theta/2); sthe2 = sin(theta/2);
  cphi2 = cos(phi/2); sphi2 = sin(phi/2);
  cpsi2 = cos(psi/2); spsi2 = sin(psi/2);

  a =  cthe2*cphi2*cpsi2 + sthe2*sphi2*spsi2;
  b =  cthe2*sphi2*cpsi2 + sthe2*cphi2*spsi2;
  c =  sthe2*cphi2*cpsi2 - cthe2*sphi2*spsi2;
  d = -sthe2*sphi2*cpsi2 + cthe2*cphi2*spsi2 ;
  
  qua_vec = [a b c d];
  