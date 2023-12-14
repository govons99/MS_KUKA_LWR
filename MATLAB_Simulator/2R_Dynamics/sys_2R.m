function dq=sys_2R(t,x,K)

global l1 l2 m1 m2 Phi Gamma

a0 = 1.13;
a1 = -0.026;  
b1 = -0.24;  
f = 5; 

w1 = a0 + a1*cos(t*f) + b1*sin(t*f);

a0 = -1;
a1 = 0.25;
b1 = 0.34;
f = 5;
                   
w2 = a0 + a1*cos(t*f) + b1*sin(t*f);

% w1 = cos(t);
% w2 = -cos(t);

e = [x(1)-w1;
     x(2)-w2];

u = K*[x(1);x(2);x(3);x(4);x(5:end)];

if (abs(u(1))>=15000)
    u(1) = sign(u(1))*15000;
end

if (abs(u(2))>=15000)
    u(2) = sign(u(2))*15000;
end

% B = inertia(x(1),x(2));
% S = coriolis(x(1),x(2),x(3),x(4));
% 
% dx = [x(3);x(4);inv(B)*(-S*x(3:4)+u)];

M11 = l2^2*m2+2*l1*l2*m2*cos(x(2))+l1^2*(m1+m2);
M12 = l2^2*m2+l1*l2*m2*cos(x(2));
M21 = M12;
M22 = l2^2*m2;

B = [M11 M12;M21 M22];

C = [-m2*l1*l2*sin(x(2))*x(4)^2-2*m2*l1*l2*sin(x(2))*x(3)*x(4);
    m2*l1*l2*sin(x(2))*x(3)^2];

g = [cos(x(1)+x(2))*m2*9.81*l2+cos(x(1))*(m1+m2)*l1*9.81;
    cos(x(1)+x(2))*m2*9.81*l2];

dx = [x(3);x(4);inv(B)*(-C+u-g)];

deta = Phi*x(5:end) + Gamma*e;

dq = [dx;deta];

end

