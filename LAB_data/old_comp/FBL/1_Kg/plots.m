clc
clear
close all

q = load("Q.txt");

dq_hat = load("dQ_hat.txt");
dq = load("dQ.txt");

q_ref = load("Qref.txt");

error = load("error.txt");

dt = 0.005;
dim = max(size(q));
index = 0.0;
time = zeros(1,dim);

for i=1:dim
    time(i) = index;
    index = index+0.005;
end

%% reference

t = 0:0.0001:600;

ref = [0.4*cos(t);-0.4*cos(t)];
ref_dot = [-0.4*sin(t);0.4*sin(t)];
ref_ddot = [-0.4*cos(t);0.4*cos(t)];

figure()
hold on; grid on;
plot(time(1:end-1),error(:,1),'linewidth',2)
plot(time(1:end-1),error(:,2),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_e [rad]")
legend("\theta_{e1}","\theta_{e2}",'location','se')
%title('error')

% axes('Position',[.73 .73 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,100000:end-1),error(100000:end,1),'b','linewidth',1.5);
% plot(time(1,100000:end-1),error(100000:end,2),'r','linewidth',1.5);

%% Joint position

figure()
hold on; grid on;
plot(time,q(:,2),'b','linewidth',2)
plot(time,q(:,4),'r','linewidth',2)
plot(time,ref(1,1:dim),'--b','linewidth',2)
plot(time,ref(2,1:dim),'--r','linewidth',2)
%title('joint position');
ylim([-0.7,0.7])
xlabel("t [s]")
ylabel("\theta [rad]")
legend('\theta_1','\theta_2','\theta_1^*','\theta_2^*','NumColumns',2);

%% joint speed

figure()
hold on; grid on;
plot(time,dq_hat(:,2));
plot(time,dq_hat(:,4));
title('joint speed (estimated)');
legend('dq1','dq2');

%% control

m1 = 2.6;
m2 = 2.6;
l1 = 0.5;
l2 = 0.5;

P_Gain = 70.0;
D_Gain = 20.0;

for i = 1:max(size(q))
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q(i,4))*dq(i,4)^2-2*m2*l1*l2*sin(q(i,4))*dq(i,2)*dq(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q(i,4))*dq(i,2)^2;

	Gravity_2R(1,1) = cos(q(i,2)+q(i,4))*m2*9.81*l2 + cos(q(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q(i,2)+q(i,4))*m2*9.81*l2;

    control(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q(i,2);q(i,4)]) + D_Gain * (ref_dot(:,i) - [dq(i,2);dq(i,4)])) + Coriolis_2R + Gravity_2R;

end

figure()
hold on; grid on;
plot(time,control','linewidth',2)
xlabel("t [s]")
ylabel("u [Nm]")
legend("u_1","u_2",'Location','nw')
%title('control')

% axes('Position',[.2 .75 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,1:4000),control(1,1:4000),'b','linewidth',1.5);
% plot(time(1,1:4000),control(2,1:4000),'r','linewidth',1.5);


