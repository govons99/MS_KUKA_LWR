function g=gravity(q1,q2)

global l1 c1x c2x m1 m2 g0

g1 = g0*m2*(c2x*cos(q1+q2)+l1*cos(q1))+c1x*g0*m1*cos(q1);
g2 = c2x*g0*m2*cos(q1+q2);
g = [g1;g2];

end

