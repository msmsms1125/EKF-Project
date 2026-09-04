function [ qua ] = inner_angle( a, b )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here    
    sum_ab = (a + b);
    norm_ab = norm(sum_ab);
    if norm_ab > 1e-6
        c = sum_ab/norm_ab;
        qua = [dot(a, c); cross(a, c)];
    else
        qua = [0; 1; 0; 0];
    end

end

