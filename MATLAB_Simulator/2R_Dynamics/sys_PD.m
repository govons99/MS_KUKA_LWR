function dx=sys_PD(t,x,Kp,Kd)

global l1 l2 m1 m2

u = -Kp*[x(1);x(2)]-Kd*[x(3);x(4)];

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

dx = [x(3);x(4);inv(B)*(-C+u)];

end

