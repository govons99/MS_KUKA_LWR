function S=coriolis(q1,q2,dq1,dq2)

global l1 c2x m2

S1 = -c2x*dq2*l1*m2*sin(q2);
S2 = -c2x*dq1*l1*m2*sin(q2)-c2x*dq2*l1*m2*sin(q2);
S3 = c2x*dq1*l1*m2*sin(q2);
S4 = 0;
S = [S1 S2;S3 S4];
    
end

