%% REFERENCE

w1 = 0.4*cos(Time_2R);
w2 = -0.4*cos(Time_2R);

qd = [w1;w2];

w1_dot = -0.4*sin(Time_2R);
w2_dot = 0.4*sin(Time_2R);

qd_dot = [w1_dot;w2_dot];

w1_ddot = -0.4*cos(Time_2R);
w2_ddot = 0.4*cos(Time_2R);

qd_ddot = [w1_ddot;w2_ddot];

%% COMPUTED TORQUE

P_Gain = 70.0;
D_Gain = 20.0;

control = Mass_2R*( qd_ddot + P_Gain * (qd - pos_2R) + D_Gain * (qd_dot - vel_2R)) + Coriolis_2R + Gravity_2R;

%% SLIDING MODE CONTROL

lambda = 8.0;
k = 150;
phi = [0.09,0.03;

vel_error = vel_2R-qd_dot;

sr = qd_dot - lambda*( pos_2R - qd );
s = vel_2R - sr;
sr_dot = qd_ddot - lambda*vel_error;

control = Mass_2R*sr_dot + Coriolis_fact_2R*sr + Gravity_2R - k*switching(s,phi);

%% Function useful for the sliding

function y = switching(s, phi)

val1 = s(1)/phi(1);
val2 = s(2)/phi(2);

res1 = 0.0;
res2 = 0.0;

if ( abs(val1)>=1 )
{
res1 = sign(val1);
}
else
{
res1 = val1;
}

if ( abs(val2)>=1 )
{
res2 = sign(val2);
}
else
{
res2 = val2;
}

y=[res1;res2];

end
