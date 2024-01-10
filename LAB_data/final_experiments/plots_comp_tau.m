clc
clear
close all

error_dar_tau2 = load("./tau=2/no_mass/error.txt");
error_dar_tau4 = load("./tau=4/error.txt");

control_dar_tau2 = load("./tau=2/no_mass/control_DAR.txt");
control_dar_tau4 = load("./tau=4/control_DAR.txt");

dt = 0.005;
dim = max(size(error_dar_tau4));
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

subplot(211)
hold on; grid on;
plot(time(1:end),control_dar_tau2(:,1),time(1:end),control_dar_tau4(:,1),'linewidth',2)
xlabel("t [s]")
ylabel("u_1 [Nm]")
xlim([0,100])
legend("\tau=2","\tau=4",'Location','se')

subplot(212)
hold on; grid on;
plot(time(1:end),control_dar_tau2(:,2),time(1:end),control_dar_tau4(:,2),'linewidth',2)
xlabel("t [s]")
ylabel("u_2 [Nm]")
xlim([0,100])
%legend("DAR","SMC","FBL",'Location','nw')

%title('control')
%saveas(gcf,'control.eps','epsc')

figure()

l = 300/dt;

subplot(211)
hold on; grid on;
plot(time(1:l),error_dar_tau2(1:l,1),time(1:l),error_dar_tau4(1:l,1),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e1} [rad]")
%xlim([0,100])
legend("\tau=2","\tau=4",'Location','se')

% axes('Position',[.73 .73 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,80000:end),error_dar_tau2(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end),error_dar_tau4(80000:end,1),'linewidth',1.5);

subplot(212)
hold on; grid on;
plot(time(1:l),error_dar_tau2(1:l,2),time(1:l),error_dar_tau4(1:l,2),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e2} [rad]")
%xlim([0,100])

% axes('Position',[.73 .26 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,80000:end),error_dar_tau2(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end),error_dar_tau4(80000:end,1),'linewidth',1.5);

saveas(gcf,'error_comp_tau.eps','epsc')
saveas(gcf,'error_comp_tau.fig')


