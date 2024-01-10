clc
clear
close all

error_dar = load("./tau=2/2_kg/error.txt");
error_smc = load("./SMC_no_matched/2_kg/error.txt");
error_fbl = load("./FBL_no_matched/2_kg/error.txt");

q_dar = load("./tau=2/2_kg/Q.txt");
q_smc = load("./SMC_no_matched/2_kg/Q.txt");

q = load("./FBL_no_matched/2_kg/Q.txt");
dq_hat = load("./FBL_no_matched/2_kg/dQ_hat.txt");

m1 = 2.6;
m2 = 2.6;
l1 = 0.5;
l2 = 0.5;

P_Gain = 70.0;
D_Gain = 20.0;

control_dar = load("./tau=2/2_kg/control_DAR.txt");
control_smc = load("./SMC_no_matched/2_kg/control.txt");

dt = 0.005;
dim = max(size(q));
index = 0.0;
time = zeros(1,dim);

for i=1:dim
    time(i) = index;
    index = index+0.005;
end

id = 300/dt;

%% reference

t = 0:0.0001:600;

ref = [0.4*cos(t);-0.4*cos(t)];
ref_dot = [-0.4*sin(t);0.4*sin(t)];
ref_ddot = [-0.4*cos(t);0.4*cos(t)];

for i = 1:max(size(q))-1
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q(i,4))*dq_hat(i,4)^2-2*m2*l1*l2*sin(q(i,4))*dq_hat(i,2)*dq_hat(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q(i,4))*dq_hat(i,2)^2;

	Gravity_2R(1,1) = cos(q(i,2)+q(i,4))*m2*9.81*l2 + cos(q(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q(i,2)+q(i,4))*m2*9.81*l2;

    control_fbl(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q(i,2);q(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat(i,2);dq_hat(i,4)])) + Coriolis_2R + Gravity_2R;

end

figure()
hold on; grid on;
plot(time(1:id),q_dar(1:id,2),time(1:id),q_dar(1:id,4), ...
     time(1:id),q_smc(1:id,2),time(1:id),q_smc(1:id,4), ...
     time(1:id),q(1:id,2),time(1:id),q(1:id,4), ...
     time(1:id),ref(1,1:id),'k--',time(1:id),ref(2,1:id),'k--','linewidth',2)
xlabel("t [s]")
ylabel("x_{\theta} [rad]")
%xlim([0,100])
legend("x_{\theta 1} DAR","x_{\theta 2} DAR", ...
       "x_{\theta 1} SMC","x_{\theta 2} SMC", ...
       "x_{\theta 1} FBL","x_{\theta 2} FBL", ...
       'Location','se','NumColumns',3)


figure()

subplot(211)
hold on; grid on;
plot(time(1:id),control_dar(1:id,1),time(1:id),control_smc(1:id,1),time(1:id),control_fbl(1,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_1 [Nm]")
%xlim([0,100])
legend("DAR","SMC","FBL",'Location','se')

subplot(212)
hold on; grid on;
plot(time(1:id),control_dar(1:id,2),time(1:id),control_smc(1:id,2),time(1:id),control_fbl(2,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_2 [Nm]")
%xlim([0,100])
%legend("DAR","SMC","FBL",'Location','nw')

%title('control')
%saveas(gcf,'control.eps','epsc')

figure()

subplot(211)
hold on; grid on;
plot(time(1:id),error_dar(1:id,1),time(1:id),error_smc(1:id,1),time(1:id),error_fbl(1:id,1),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e1} [rad]")
%xlim([0,100])
legend("DAR","SMC","FBL",'Location','se')

% axes('Position',[.73 .73 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,80000:end-1),error_dar(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end-1),error_smc(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end-1),error_fbl(80000:end,1),'linewidth',1.5);

subplot(212)
hold on; grid on;
plot(time(1:id),error_dar(1:id,2),time(1:id),error_smc(1:id,2),time(1:id),error_fbl(1:id,2),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e2} [rad]")
%xlim([0,100])

% axes('Position',[.73 .26 .15 .15])
% box on;
% hold on; grid on;
% plot(time(1,80000:end-1),error_dar(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end-1),error_smc(80000:end,1),'linewidth',1.5);
% plot(time(1,80000:end-1),error_fbl(80000:end,1),'linewidth',1.5);

%saveas(gcf,'error.eps','epsc')


