clc
clear
close all

q = load("Q.txt");

dq_hat = load("dQ_hat.txt");
dq = load("dQ.txt");

q_ref = load("Qref.txt");

error = load("error.txt");

control = load("control.txt");

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

axes('Position',[.73 .73 .15 .15])
box on;
hold on; grid on;
plot(time(1,80000:end-1),error(80000:end,1),'b','linewidth',1.5);
plot(time(1,80000:end-1),error(80000:end,2),'r','linewidth',1.5);

saveas(gcf,'error_SMC.eps','epsc')

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
saveas(gcf,'pos_SMC.eps','epsc')

%% joint speed

figure()
hold on; grid on;
plot(time,dq(:,2));
plot(time,dq(:,4));
plot(time,dq_hat(:,2),'LineWidth',2);
plot(time,dq_hat(:,4),'LineWidth',2');
%title('joint speed (estimated)');
legend('$\dot{q}_1$','$\dot{q}_2$','$\hat{\dot{q}}_1$','$\hat{\dot{q}}_2$','interpreter','latex','NumColumns',2);
saveas(gcf,'vel_SMC.eps','epsc')

%% control

figure()
hold on; grid on;
plot(time(1:end-1),control,'linewidth',2)
xlabel("t [s]")
ylabel("u [Nm]")
legend("u_1","u_2",'Location','nw')
%title('control')
saveas(gcf,'control_SMC.eps','epsc')

% axes('Position',[.2 .75 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,1:4000),control(1,1:4000),'b','linewidth',1.5);
% plot(time(1,1:4000),control(2,1:4000),'r','linewidth',1.5);


