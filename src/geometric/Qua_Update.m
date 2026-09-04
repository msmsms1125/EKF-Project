function [ x ] = Qua_Update( qua_a, qua_b )
%Quaternion update

x = [qua_a(1)*qua_b(1) - qua_a(2)*qua_b(2) - qua_a(3)*qua_b(3) - qua_a(4)*qua_b(4)
    qua_a(1)*qua_b(2) + qua_a(2)*qua_b(1) + qua_a(3)*qua_b(4) - qua_a(4)*qua_b(3)
    qua_a(1)*qua_b(3) + qua_a(3)*qua_b(1) + qua_a(4)*qua_b(2) - qua_a(2)*qua_b(4)
    qua_a(1)*qua_b(4) + qua_a(4)*qua_b(1) + qua_a(2)*qua_b(3) - qua_a(3)*qua_b(2)];
x = x/norm(x);
end

