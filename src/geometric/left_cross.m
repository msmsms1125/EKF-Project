function [M] = left_cross(V)
%%%% [V X]

    M = zeros(3,3);
    
    M(1,2) = -V(3);
    M(2,1) = +V(3);
    
    M(1,3) = +V(2);
    M(3,1) = -V(2);
    
    M(2,3) = -V(1);
    M(3,2) = +V(1);

