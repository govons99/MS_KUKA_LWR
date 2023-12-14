clc
clear 
close all

global l1 l2 c1x c2x c1y c2y m1 m2 I1zz I2zz g0 F
global gain dt sum1 sum2 fault force

% Physical parameters

g0 = 9.81;

l1 = 0.5;
l2 = 0.5;
m1 = 5.4;
m2 = 3.6;
c1x = 0.5;
c1y = 0;
c1z = 0;
c2x = 0.5;
c2y = 0;
c2z = 0;
I1zz = 1/12*m1*l1^2;
I2zz = 1/12*m2*l2^2;
%F=0*0.1*[1 2;2 5];
F=0.1*[1 2;2 5];

% Pssible faulty situations:
% fault = 0 ---> no fault
% fault = 1 ---> the first motor looses 90% of its power at t=5s
% fault = 2 ---> the second motor looses 90% of its power at t=5s
% fault = 3 ---> at t=5s, the first motor actuates only 80% of the 
%                commanded torque and at t=10s the second motor exhibits 
%                the same type of faulty behavior
% fault = 4 ---> external force F=[0;-500] applied to the CoM of the first
%                link from t=5s up to 10s
% fault = 5 ---> external force F=[0;-500] applied to the CoM of the second
%                link from t=5s up to 10s

fault = 5;

force = [0; -300];

% Residual parameters
gain = 50*eye(2);
dt = 0.0001;
sum1 = [0;0];
sum2 = [0;0];

% INITIAL CONDITIONS
x0=[-pi/2;0;0;0];

tspan = 0:dt:30;
[t_ode,x_ode] = ode45(@robot_sys_prova,tspan,x0);

q = x_ode(:,1:2)';
q1 = q(1,:);
q2 = q(2,:);
q_dot = x_ode(:,3:4)';

% Positions plot
f1 = figure(1);
title('Joint angular position');
hold on
plot(t_ode,q1,t_ode,q2,'linewidth',2)
grid on;
legend('joint position q_1','joint position q_2','location','SouthWest');
xlabel('time [s]');
ylabel('rad');

% Velocity plot
f2 = figure(2);
title('Joint angular velocity');
hold on
plot(t_ode,q_dot(1,:),t_ode,q_dot(2,:),'linewidth',2)
grid on
legend('joint velocity $\dot{q}_1$','joint velocity $\dot{q}_2$','interpreter','latex','location','SouthWest');
xlabel('time [s]');
ylabel('rad/s');

% % Evaluation of the residual
% 
% [~,dim] = size(t_ode');
% 
% r = zeros(2,dim);
% u_n = zeros(2,dim); % nominal control
% u_f = zeros(2,dim); % external torque
% 
% for i=1:dim
%     r(:,i) = discrete_res(q(:,i),q_dot(:,i),t_ode(i),r(:,1:i),i);
%     q_i = q(:,i);
%     q1 = q_i(1);
%     q2 = q_i(2);
%     g_n = gravity(q1,q2);
%     u_n(:,i) = g_n+[0;1]+[sin(t_ode(i));sin(2*t_ode(i))];
%     u_f(:,i) = faultcontrol(u_n(:,i),t_ode(i),q_i,fault);
% end
% 
% a = max(abs(r(1,:)));
% b = max(abs(r(2,:)));
% y_max = max(a,b);
% 
% f3 = figure(3);
% title('Residual on the first joint');
% hold on
% plot(t_ode,r(1,:),'linewidth',2);
% grid on;
% ylim([-y_max,y_max]);
% 
% f4 = figure(4);
% title('Residual on the second joint');
% hold on
% plot(t_ode,r(2,:),'linewidth',2)
% grid on;
% ylim([-y_max,y_max]);
% 
% f5 = figure(5);
% subplot(211)
% hold on
% title('First joint control input')
% plot(t_ode,u_n(1,:),'linewidth',2)
% grid on
% legend('un_1');
% subplot(212)
% hold on
% title('Second joint control input')
% plot(t_ode,u_n(2,:),'linewidth',2)
% grid on
% legend('un_2');
% 
% f6 = figure(6);
% subplot(211)
% hold on
% title('First joint actual control input: \tau=u_n+u_f')
% plot(t_ode,u_f(1,:)+u_n(1,:),'linewidth',2)
% grid on
% legend('\tau_1');
% subplot(212)
% hold on
% title('Second joint actual control input: \tau=u_n+u_f')
% plot(t_ode,u_f(2,:)+u_n(2,:),'linewidth',2)
% grid on
% legend('\tau_2');
% 
% f7 = figure(7);
% 
% subplot(211)
% hold on
% title('First component')
% plot(t_ode,r(1,:),'--',t_ode,u_f(1,:),'linewidth',2)
% grid on
% legend('residual','$\tau_{ext}$','location','SouthWest', 'interpreter', 'latex')
% ylim([-800 800])
% xlabel('time [s]');
% 
% subplot(212)
% hold on
% title('Second component')
% plot(t_ode,r(2,:),'--',t_ode,u_f(2,:),'linewidth',2)
% grid on
% legend('residual','$\tau_{ext}$','location','SouthWest', 'interpreter', 'latex')
% ylim([-800 800])
% xlabel('time [s]');
% 
% sgtitle('Residual');

% saving the images in the .eps format
% saveas(f1,'Joint_angular_position.eps','epsc');
% saveas(f2,'Joint_angular_velocity.eps','epsc');
% saveas(f3,'Residual_first_joint.eps','epsc');
% saveas(f4,'Residual_second_joint.eps','epsc');
% saveas(f5,'Nominal_control.eps','epsc');
% saveas(f6,'Actual_control.eps','epsc');
% saveas(f7,'Comparison5.eps','epsc');
% saveas(f7,'Comparison5.png','png');





