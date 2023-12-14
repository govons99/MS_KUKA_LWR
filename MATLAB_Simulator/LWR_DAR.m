%% INIT
clc;
clear;
close all;

DAR_parameters;

global l1 l2 m1 m2

m1 = 5.4;
m2 = 2.6;
l1 = 0.5;
l2 = 0.5;

q_2r = [q0(2);q0(4)];
dq_2r = [q0(9);q0(11)];
eta0 = zeros(6,1);

tspan = 0:DeltaT:tf;
[t_ode,x_ode] = ode45(@(t,z) sys_2R(t,z,K),tspan,[q_2r;dq_2r;eta0]);

%[t_PD,x_PD] = ode45(@(t,z) sys_PD(t,z,Kp,Kd),tspan,[q_2r;dq_2r]);

%general_plots;

q = x_ode(:,1:2)';
q1 = q(1,:);
q2 = q(2,:);
q_dot = x_ode(:,3:4)';
eta = x_ode(:,5:end)';

q_new = q0.*ones(max(size(t_ode)),14);

q_new(:,2) = q1;
q_new(:,4) = q2;
q_new(:,9) = q_dot(1,:);
q_new(:,11) = q_dot(2,:);

joints = vertcat(joints,q_new);

u = K*[q1;q2;q_dot;eta];

for i=1:max(size(u))
    if (abs(u(1,i))>=15000)
    u(1,i) = sign(u(1,i))*15000;
    end
    
    if (abs(u(2,i))>=15000)
        u(2,i) = sign(u(2,i))*15000;
    end
end

% 
% u_PD = -Kp*x_PD(:,1:2)-Kd*x_PD(:,3:4);

d = date;
save("workspace_"+d);

a0 = 1.13;
a1 = -0.026;  
b1 = -0.24;  
f = 5; 

w1 = a0 + a1*cos(t_ode*f) + b1*sin(t_ode*f);

a0 = -1;
a1 = 0.25;
b1 = 0.34;
f = 5;
                   
w2 = a0 + a1*cos(t_ode*f) + b1*sin(t_ode*f);

% Positions plot
figure();
title('Joint angular position');
hold on
plot(t_ode,q1,t_ode,q2,'linewidth',2)
plot(t_ode,w1,'b--',t_ode,w2,'r--')
grid on;
legend('joint position q_1','joint position q_2','location','se');
xlabel('time [s]');
ylabel('rad');

% Velocity plot
figure();
title('Joint angular velocity');
hold on
plot(t_ode,q_dot(1,:),t_ode,q_dot(2,:),'linewidth',2)
grid on
legend('joint velocity $\dot{q}_1$','joint velocity $\dot{q}_2$','interpreter','latex','location','se');
xlabel('time [s]');
ylabel('rad/s');

% Internal model
figure()
title('internal model')
hold on; grid on;
plot(t_ode,eta);

% Control plot
figure();
title('Control profile');
hold on
plot(t_ode,u,'linewidth',2)
grid on
legend('u1','u2','location','se');
xlabel('time [s]');
ylabel('Nm');

% % Positions plot
% figure();
% title('PD Joint angular position');
% hold on
% plot(t_PD,x_PD(:,1),t_PD,x_PD(:,2),'linewidth',2)
% grid on;
% legend('joint position q_1','joint position q_2','location','se');
% xlabel('time [s]');
% ylabel('rad');
% 
% % Velocity plot
% figure();
% title('PD Joint angular velocity');
% hold on
% plot(t_PD,x_PD(:,3),t_PD,x_PD(:,4),'linewidth',2)
% grid on
% legend('joint velocity $\dot{q}_1$','joint velocity $\dot{q}_2$','interpreter','latex','location','se');
% xlabel('time [s]');
% ylabel('rad/s');
% 
% % Control plot
% figure();
% title('PD Control profile');
% hold on
% plot(t_PD,u_PD,'linewidth',2)
% grid on
% legend('u1','u2','location','se');
% xlabel('time [s]');
% ylabel('Nm');

