function qua_vec = ang2qua(three_angle)
 % three angle to quaternion
 
 ang = norm(three_angle);
 
 if abs(ang) > 0.0001
     
     qua_vec = [cos(ang/2); sin(ang/2)*(three_angle/ang)];
     
 else
     
     qua_vec = [cos(ang/2); three_angle/2];
     
 end
 
 qua_vec = qua_vec/norm(qua_vec);
     
     
  
  
  