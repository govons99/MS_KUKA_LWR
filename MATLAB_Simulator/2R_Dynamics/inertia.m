function B=inertia(q1,q2)

global l1 c1x c2x m1 m2 I1zz I2zz 

B1 = m1*c1x^2+m2*(l1^2+c2x^2+2*l1*c2x*cos(q2))+I1zz+I2zz;
B2 = m2*(c2x^2+l1*c2x*cos(q2))+I2zz;
B3 = m2*(c2x^2+l1*c2x*cos(q2))+I2zz;
B4 = m2*c2x^2+I2zz;
B = [B1 B2;B3 B4];

end

