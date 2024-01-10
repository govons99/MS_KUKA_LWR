clc
clear
close all

%% loading values

% no mass values

error_dar_no = load("./tau=2/no_mass/error.txt");
error_smc_no = load("./SMC_no_matched/no_mass/error.txt");
error_fbl_no = load("./FBL_no_matched/no_mass/error.txt");

q_dar_no = load("./tau=2/no_mass/Q.txt");
q_smc_no = load("./SMC_no_matched/no_mass/Q.txt");
q_fbl_no = load("./FBL_no_matched/no_mass/Q.txt");

dq_hat_no = load("./FBL_no_matched/no_mass/dQ_hat.txt");

control_dar_no = load("./tau=2/no_mass/control_DAR.txt");
control_smc_no = load("./SMC_no_matched/no_mass/control.txt");

% 2 kg values

error_dar_2 = load("./tau=2/2_kg/error.txt");
error_smc_2 = load("./SMC_no_matched/2_kg/error.txt");
error_fbl_2 = load("./FBL_no_matched/2_kg/error.txt");

q_dar_2 = load("./tau=2/2_kg/Q.txt");
q_smc_2 = load("./SMC_no_matched/2_kg/Q.txt");
q_fbl_2 = load("./FBL_no_matched/2_kg/Q.txt");

dq_hat_2 = load("./FBL_no_matched/2_kg/dQ_hat.txt");

control_dar_2 = load("./tau=2/2_kg/control_DAR.txt");
control_smc_2 = load("./SMC_no_matched/2_kg/control.txt");

m1 = 2.6;
m2 = 2.6;
l1 = 0.5;
l2 = 0.5;

P_Gain = 70.0;
D_Gain = 20.0;

dt = 0.005;
dim = max(size(q_fbl_no));
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

%% plots

for i = 1:max(size(q_fbl_no))-1
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_no(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_no(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_no(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_no(i,4))*dq_hat_no(i,4)^2-2*m2*l1*l2*sin(q_fbl_no(i,4))*dq_hat_no(i,2)*dq_hat_no(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_no(i,4))*dq_hat_no(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_no(i,2)+q_fbl_no(i,4))*m2*9.81*l2 + cos(q_fbl_no(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_no(i,2)+q_fbl_no(i,4))*m2*9.81*l2;

    control_fbl_no(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_no(i,2);q_fbl_no(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_no(i,2);dq_hat_no(i,4)])) + Coriolis_2R + Gravity_2R;

end

for i = 1:max(size(q_fbl_2))-1
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_2(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_2(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_2(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_2(i,4))*dq_hat_2(i,4)^2-2*m2*l1*l2*sin(q_fbl_2(i,4))*dq_hat_2(i,2)*dq_hat_2(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_2(i,4))*dq_hat_2(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_2(i,2)+q_fbl_2(i,4))*m2*9.81*l2 + cos(q_fbl_2(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_2(i,2)+q_fbl_2(i,4))*m2*9.81*l2;

    control_fbl_2(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_2(i,2);q_fbl_2(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_2(i,2);dq_hat_2(i,4)])) + Coriolis_2R + Gravity_2R;

end

%%
id = 300/dt;

figure()
tiledlayout('flow')

nexttile
hold on; grid on;
plot(time(1:id),q_dar_no(1:id,2),time(1:id),q_dar_no(1:id,4), ...
     time(1:id),q_smc_no(1:id,2),time(1:id),q_smc_no(1:id,4), ...
     time(1:id),q_fbl_no(1:id,2),time(1:id),q_fbl_no(1:id,4), ...
     time(1:id),ref(1,1:id),'k--',time(1:id),ref(2,1:id),'k--','linewidth',2)
xlabel("t [s]")
ylabel("x_{\theta} [rad]")
%xlim([0,100])
title('no mass')
hold off;
% legend("x_{\theta 1} DAR","x_{\theta 2} DAR", ...
%        "x_{\theta 1} SMC","x_{\theta 2} SMC", ...
%        "x_{\theta 1} FBL","x_{\theta 2} FBL", ...
%        'Location','se','NumColumns',3)

nexttile
hold on; grid on;
plot(time(1:id),q_dar_2(1:id,2),time(1:id),q_dar_2(1:id,4), ...
     time(1:id),q_smc_2(1:id,2),time(1:id),q_smc_2(1:id,4), ...
     time(1:id),q_fbl_2(1:id,2),time(1:id),q_fbl_2(1:id,4), ...
     time(1:id),ref(1,1:id),'k--',time(1:id),ref(2,1:id),'k--','linewidth',2)
xlabel("t [s]")
ylabel("x_{\theta} [rad]")
title('2 Kg')
%xlim([0,100])
hold off;

lg = legend("x_{\theta 1} DAR","x_{\theta 2} DAR", ...
       "x_{\theta 1} SMC","x_{\theta 2} SMC", ...
       "x_{\theta 1} FBL","x_{\theta 2} FBL", ...
       'Location','se','NumColumns',3);
lg.Layout.Tile = 'north';

saveas(gcf,'pos_no_mathced.eps','epsc')
saveas(gcf,'pos_no_mathced.fig','fig')

%%

figure()
tiledlayout(2,2)

% no mass
nexttile
hold on; grid on;
plot(time(1:id),control_dar_no(1:id,1),time(1:id),control_smc_no(1:id,1),time(1:id),control_fbl_no(1,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_1 [Nm]")
ylim([-150,100])
lg = legend("DAR","SMC","FBL",'Location','se','NumColumns',3);
lg.Layout.Tile = 'north';
title('no mass')

% 2 kg
nexttile
hold on; grid on;
plot(time(1:id),control_dar_2(1:id,1),time(1:id),control_smc_2(1:id,1),time(1:id),control_fbl_2(1,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_1 [Nm]")
ylim([-150,100])
%legend("DAR","SMC","FBL",'Location','se')
title('2 Kg')

nexttile
hold on; grid on;
plot(time(1:id),control_dar_no(1:id,2),time(1:id),control_smc_no(1:id,2),time(1:id),control_fbl_no(2,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_2 [Nm]")
ylim([-40,60])
%legend("DAR","SMC","FBL",'Location','nw')

nexttile
hold on; grid on;
plot(time(1:id),control_dar_2(1:id,2),time(1:id),control_smc_2(1:id,2),time(1:id),control_fbl_2(2,1:id),'linewidth',2)
xlabel("t [s]")
ylabel("u_2 [Nm]")
ylim([-40,60])

saveas(gcf,'u_no_mathced.eps','epsc')
saveas(gcf,'u_no_mathced.fig','fig')

%title('control')
%saveas(gcf,'control.eps','epsc')

%%

figure()
tiledlayout(2,2)

nexttile
hold on; grid on;
plot(time(1:id),error_dar_no(1:id,1),time(1:id),error_smc_no(1:id,1),time(1:id),error_fbl_no(1:id,1),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e1} [rad]")
lg = legend("DAR","SMC","FBL",'Location','nw','NumColumns',3);
lg.Layout.Tile = 'north';
ylim([-0.3,0.1])
title('no mass')
hold off;

nexttile
hold on; grid on;
plot(time(1:id),error_dar_2(1:id,1),time(1:id),error_smc_2(1:id,1),time(1:id),error_fbl_2(1:id,1),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e1} [rad]")
ylim([-0.3,0.1])
%legend("DAR","SMC","FBL",'Location','nw','numcolumns',3)
title('2 Kg')
hold off;

nexttile
hold on; grid on;
plot(time(1:id),error_dar_no(1:id,2),time(1:id),error_smc_no(1:id,2),time(1:id),error_fbl_no(1:id,2),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e2} [rad]")
hold off;

nexttile
hold on; grid on;
plot(time(1:id),error_dar_2(1:id,2),time(1:id),error_smc_2(1:id,2),time(1:id),error_fbl_2(1:id,2),'linewidth',2)
xlabel("t [s]")
ylabel("\theta_{e2} [rad]")
hold off;

saveas(gcf,'err_no_mathced.eps','epsc')
saveas(gcf,'err_no_mathced.fig','fig')


