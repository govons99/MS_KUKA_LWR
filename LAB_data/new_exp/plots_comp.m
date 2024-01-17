clc
clear
close all


%% wrong model

% matched values
q_dar_wm = load("./wrong model/DAR_matched/Q.txt");
q_smc_wm = load("./wrong model/SMC_matched/Q.txt");
q_fbl_wm = load("./wrong model/FBL_matched/Q.txt");

u_dar_wm = load("./wrong model/DAR_matched/control_DAR.txt");
u_smc_wm = load("./wrong model/SMC_matched/control.txt");

dq_hat_wm = load("./wrong model/FBL_matched/dQ_hat.txt");

e_dar_wm = load("./wrong model/DAR_matched/error.txt");
e_smc_wm = load("./wrong model/SMC_matched/error.txt");
e_fbl_wm = load("./wrong model/FBL_matched/error.txt");

% unmatched values
q_dar_wu = load("./wrong model/DAR_unmatched/Q.txt");
q_smc_wu = load("./wrong model/SMC_unmatched/Q.txt");
q_fbl_wu = load("./wrong model/FBL_unmatched/Q.txt");

u_dar_wu = load("./wrong model/DAR_unmatched/control_DAR.txt");
u_smc_wu = load("./wrong model/SMC_unmatched/control.txt");

e_dar_wu = load("./wrong model/DAR_unmatched/error.txt");
e_smc_wu = load("./wrong model/SMC_unmatched/error.txt");
e_fbl_wu = load("./wrong model/FBL_unmatched/error.txt");

dq_hat_wu = load("./wrong model/FBL_unmatched/dQ_hat.txt");

%% no mass values

% matched
e_dar_nom = load("./tau=4/no_mass_matched/error.txt");
e_smc_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/no_mass/error.txt");
e_fbl_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/no_mass/error.txt");

q_dar_nom = load("./tau=4/no_mass_matched/Q.txt");
q_smc_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/no_mass/Q.txt");
q_fbl_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/no_mass/Q.txt");

dq_hat_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/no_mass/dQ_hat.txt");

u_dar_nom = load("./tau=4/no_mass_matched/control_DAR.txt");
u_smc_nom = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/no_mass/control.txt");

% unmatched
e_dar_nou = load("./tau=4/no_mass_unmatched/error.txt");
e_smc_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/no_mass/error.txt");
e_fbl_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/no_mass/error.txt");

q_dar_nou = load("./tau=4/no_mass_unmatched/Q.txt");
q_smc_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/no_mass/Q.txt");
q_fbl_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/no_mass/Q.txt");

dq_hat_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/no_mass/dQ_hat.txt");

u_dar_nou = load("./tau=4/no_mass_unmatched/control_DAR.txt");
u_smc_nou = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/no_mass/control.txt");

%% 2 kg values

% matched
e_dar_2m = load("./tau=4/2kg_matched/error.txt");
e_smc_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/2_Kg/error.txt");
e_fbl_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/2_Kg/error.txt");

q_dar_2m = load("./tau=4/2kg_matched/Q.txt");
q_smc_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/2_Kg/Q.txt");
q_fbl_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/2_Kg/Q.txt");

dq_hat_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/FBL/2_Kg/dQ_hat.txt");

u_dar_2m = load("./tau=4/2kg_matched/control_DAR.txt");
u_smc_2m = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/comp/SMC/2_Kg/control.txt");

% unmatched
e_dar_2u = load("./tau=4/2kg_unmatched/error.txt");
e_smc_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/2_kg/error.txt");
e_fbl_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/2_kg/error.txt");

q_dar_2u = load("./tau=4/2kg_unmatched/Q.txt");
q_smc_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/2_kg/Q.txt");
q_fbl_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/2_kg/Q.txt");

dq_hat_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/FBL_no_matched/2_kg/dQ_hat.txt");

u_dar_2u = load("./tau=4/2kg_unmatched/control_DAR.txt");
u_smc_2u = load("/home/lorenzo/Documents/magistrale/Kuka_LWR_Controller-lorenzo/LAB_data/final_experiments/SMC_no_matched/2_kg/control.txt");

%% reference

dt = 0.005;
dim = max(size(q_fbl_wu));
index = 0.0;
time = zeros(1,dim);

for i=1:dim
    time(i) = index;
    index = index+0.005;
end

id = 300/dt;

t = 0:0.0001:600;

ref = [0.4*cos(t);-0.4*cos(t)];
ref_dot = [-0.4*sin(t);0.4*sin(t)];
ref_ddot = [-0.4*cos(t);0.4*cos(t)];

%% plots

m1 = 2.6;
m2 = 2.6;
l1 = 0.5;
l2 = 0.5;

P_Gain = 70.0;
D_Gain = 20.0;

for i = 1:max(size(q_fbl_wm))-1
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_wm(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_wm(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_wm(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_wm(i,4))*dq_hat_wm(i,4)^2-2*m2*l1*l2*sin(q_fbl_wm(i,4))*dq_hat_wm(i,2)*dq_hat_wm(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_wm(i,4))*dq_hat_wm(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_wm(i,2)+q_fbl_wm(i,4))*m2*9.81*l2 + cos(q_fbl_wm(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_wm(i,2)+q_fbl_wm(i,4))*m2*9.81*l2;

    control_fbl_wm(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_wm(i,2);q_fbl_wm(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_wm(i,2);dq_hat_wm(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_wm = control_fbl_wm';

for i = 1:max(size(q_fbl_wu))-1
    
    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_wu(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_wu(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_wu(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_wu(i,4))*dq_hat_wu(i,4)^2-2*m2*l1*l2*sin(q_fbl_wu(i,4))*dq_hat_wu(i,2)*dq_hat_wu(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_wu(i,4))*dq_hat_wu(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_wu(i,2)+q_fbl_wu(i,4))*m2*9.81*l2 + cos(q_fbl_wu(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_wu(i,2)+q_fbl_wu(i,4))*m2*9.81*l2;

    control_fbl_wu(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_wu(i,2);q_fbl_wu(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_wu(i,2);dq_hat_wu(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_wu = control_fbl_wu';

for i = 1:max(size(q_fbl_nom))-1

    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_nom(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_nom(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_nom(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_nom(i,4))*dq_hat_nom(i,4)^2-2*m2*l1*l2*sin(q_fbl_nom(i,4))*dq_hat_nom(i,2)*dq_hat_nom(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_nom(i,4))*dq_hat_nom(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_nom(i,2)+q_fbl_nom(i,4))*m2*9.81*l2 + cos(q_fbl_nom(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_nom(i,2)+q_fbl_nom(i,4))*m2*9.81*l2;

    control_fbl_nom(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_nom(i,2);q_fbl_nom(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_nom(i,2);dq_hat_nom(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_nom = control_fbl_nom';

for i = 1:max(size(q_fbl_nou))-1

    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_nou(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_nou(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_nou(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_nou(i,4))*dq_hat_nou(i,4)^2-2*m2*l1*l2*sin(q_fbl_nou(i,4))*dq_hat_nou(i,2)*dq_hat_nou(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_nou(i,4))*dq_hat_nou(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_nou(i,2)+q_fbl_nou(i,4))*m2*9.81*l2 + cos(q_fbl_nou(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_nou(i,2)+q_fbl_nou(i,4))*m2*9.81*l2;

    control_fbl_nou(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_nou(i,2);q_fbl_nou(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_nou(i,2);dq_hat_nou(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_nou = control_fbl_nou';

for i = 1:max(size(q_fbl_2m))-1

    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_2m(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_2m(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_2m(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_2m(i,4))*dq_hat_2m(i,4)^2-2*m2*l1*l2*sin(q_fbl_2m(i,4))*dq_hat_2m(i,2)*dq_hat_2m(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_2m(i,4))*dq_hat_2m(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_2m(i,2)+q_fbl_2m(i,4))*m2*9.81*l2 + cos(q_fbl_2m(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_2m(i,2)+q_fbl_2m(i,4))*m2*9.81*l2;

    control_fbl_2m(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_2m(i,2);q_fbl_2m(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_2m(i,2);dq_hat_2m(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_2m = control_fbl_2m';

for i = 1:max(size(q_fbl_2u))-1

    Mass_2R(1,1) = l2^2*m2+2*l1*l2*m2*cos(q_fbl_2u(i,4))+l1^2*(m1+m2);
	Mass_2R(1,2) = l2^2*m2+l1*l2*m2*cos(q_fbl_2u(i,4));
	Mass_2R(2,1) = l2^2*m2+l1*l2*m2*cos(q_fbl_2u(i,4));
	Mass_2R(2,2) = l2^2*m2;

	Coriolis_2R(1,1) = -m2*l1*l2*sin(q_fbl_2u(i,4))*dq_hat_2u(i,4)^2-2*m2*l1*l2*sin(q_fbl_2u(i,4))*dq_hat_2u(i,2)*dq_hat_2u(i,4);
	Coriolis_2R(2,1) = m2*l1*l2*sin(q_fbl_2u(i,4))*dq_hat_2u(i,2)^2;

	Gravity_2R(1,1) = cos(q_fbl_2u(i,2)+q_fbl_2u(i,4))*m2*9.81*l2 + cos(q_fbl_2u(i,2))*(m1+m2)*l1*9.81;
	Gravity_2R(2,1) = cos(q_fbl_2u(i,2)+q_fbl_2u(i,4))*m2*9.81*l2;

    control_fbl_2u(:,i) = Mass_2R*( ref_ddot(:,i) + P_Gain * (ref(:,i) - [q_fbl_2u(i,2);q_fbl_2u(i,4)]) + D_Gain * (ref_dot(:,i) - [dq_hat_2u(i,2);dq_hat_2u(i,4)])) + Coriolis_2R + Gravity_2R;

end

u_fbl_2u = control_fbl_2u';

%% wrong model

% position
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),q_dar_wm(1:id,2),'b',time(1:id),q_dar_wm(1:id,4),'b--', ...
     time(1:id),q_smc_wm(1:id,2),'r',time(1:id),q_smc_wm(1:id,4),'r--', ...
     time(1:id),q_fbl_wm(1:id,2),'g',time(1:id),q_fbl_wm(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

nexttile
hold on; grid on;
plot(time(1:id),q_dar_wu(1:id,2),'b',time(1:id),q_dar_wu(1:id,4),'b--', ...
     time(1:id),q_smc_wu(1:id,2),'r',time(1:id),q_smc_wu(1:id,4),'r--', ...
     time(1:id),q_fbl_wu(1:id,2),'g',time(1:id),q_fbl_wu(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

lg = legend("$DAR-x_{\theta 1}$","$DAR-x_{\theta 2}$", ...
       "$SMC-x_{\theta 1}$","$SMC-x_{\theta 2}$", ...
       "$FBL-x_{\theta 1}$","$FBL-x_{\theta 2}$", ...
       "$REF-\omega_{\theta_1}$","$REF-\omega_{\theta_2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'wrong_pos.fig','fig')

% control
figure()
tiledlayout(4,4)

nexttile(1,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_wm(1:id,1),'b',time(1:id),u_dar_wm(1:id,2),'b--', ...
     time(1:id),u_smc_wm(1:id,1),'r',time(1:id),u_smc_wm(1:id,2),'r--', ...
     time(1:id),u_fbl_wm(1:id,1),'g',time(1:id),u_fbl_wm(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

lg = legend("$DAR-u_1$","$DAR-u_2$", ...
       "$SMC-u_1$","$SMC-u_2$", ...
       "$FBL-u_1$","$FBL-u_2$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

nexttile(3,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_wm(1:id,1),'b', ...
     time(1:id),u_smc_wm(1:id,1),'r', ...
     time(1:id),u_fbl_wm(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(7,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_wm(1:id,2),'b', ...
     time(1:id),u_smc_wm(1:id,2),'r', ...
     time(1:id),u_fbl_wm(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

nexttile(9,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_wu(1:id,1),'b',time(1:id),u_dar_wu(1:id,2),'b--', ...
     time(1:id),u_smc_wu(1:id,1),'r',time(1:id),u_smc_wu(1:id,2),'r--', ...
     time(1:id),u_fbl_wu(1:id,1),'g',time(1:id),u_fbl_wu(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

nexttile(11,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_wu(1:id,1),'b', ...
     time(1:id),u_smc_wu(1:id,1),'r', ...
     time(1:id),u_fbl_wu(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(15,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_wu(1:id,2),'b', ...
     time(1:id),u_smc_wu(1:id,2),'r', ...
     time(1:id),u_fbl_wu(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

saveas(gcf,'wrong_u.fig','fig')

% error
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),e_dar_wm(1:id,1),'b',time(1:id),e_dar_wm(1:id,2),'b--', ...
     time(1:id),e_smc_wm(1:id,1),'r',time(1:id),e_smc_wm(1:id,2),'r--', ...
     time(1:id),e_fbl_wm(1:id,1),'g',time(1:id),e_fbl_wm(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3]);
hold off;

nexttile
hold on; grid on;
plot(time(1:id),e_dar_wu(1:id,1),'b',time(1:id),e_dar_wu(1:id,2),'b--', ...
     time(1:id),e_smc_wu(1:id,1),'r',time(1:id),e_smc_wu(1:id,2),'r--', ...
     time(1:id),e_fbl_wu(1:id,1),'g',time(1:id),e_fbl_wu(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3])
hold off;

lg = legend("$DAR-z_{\theta 1}$","$DAR-z_{\theta 2}$", ...
       "$SMC-z_{\theta 1}$","$SMC-z_{\theta 2}$", ...
       "$FBL-z_{\theta 1}$","$FBL-z_{\theta 2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'wrong_e.fig','fig')

%% no mass 

% position
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),q_dar_nom(1:id,2),'b',time(1:id),q_dar_nom(1:id,4),'b--', ...
     time(1:id),q_smc_nom(1:id,2),'r',time(1:id),q_smc_nom(1:id,4),'r--', ...
     time(1:id),q_fbl_nom(1:id,2),'g',time(1:id),q_fbl_nom(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

nexttile
hold on; grid on;
plot(time(1:id),q_dar_nou(1:id,2),'b',time(1:id),q_dar_nou(1:id,4),'b--', ...
     time(1:id),q_smc_nou(1:id,2),'r',time(1:id),q_smc_nou(1:id,4),'r--', ...
     time(1:id),q_fbl_nou(1:id,2),'g',time(1:id),q_fbl_nou(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

lg = legend("$DAR-x_{\theta 1}$","$DAR-x_{\theta 2}$", ...
       "$SMC-x_{\theta 1}$","$SMC-x_{\theta 2}$", ...
       "$FBL-x_{\theta 1}$","$FBL-x_{\theta 2}$", ...
       "$REF-\omega_{\theta_1}$","$REF-\omega_{\theta_2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'nomass_pos.fig','fig')

% control
figure()
tiledlayout(4,4)

nexttile(1,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_nom(1:id,1),'b',time(1:id),u_dar_nom(1:id,2),'b--', ...
     time(1:id),u_smc_nom(1:id,1),'r',time(1:id),u_smc_nom(1:id,2),'r--', ...
     time(1:id),u_fbl_nom(1:id,1),'g',time(1:id),u_fbl_nom(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

lg = legend("$DAR-u_1$","$DAR-u_2$", ...
       "$SMC-u_1$","$SMC-u_2$", ...
       "$FBL-u_1$","$FBL-u_2$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

nexttile(3,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_nom(1:id,1),'b', ...
     time(1:id),u_smc_nom(1:id,1),'r', ...
     time(1:id),u_fbl_nom(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(7,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_nom(1:id,2),'b', ...
     time(1:id),u_smc_nom(1:id,2),'r', ...
     time(1:id),u_fbl_nom(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

nexttile(9,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_nou(1:id,1),'b',time(1:id),u_dar_nou(1:id,2),'b--', ...
     time(1:id),u_smc_nou(1:id,1),'r',time(1:id),u_smc_nou(1:id,2),'r--', ...
     time(1:id),u_fbl_nou(1:id,1),'g',time(1:id),u_fbl_nou(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

nexttile(11,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_nou(1:id,1),'b', ...
     time(1:id),u_smc_nou(1:id,1),'r', ...
     time(1:id),u_fbl_nou(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(15,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_nou(1:id,2),'b', ...
     time(1:id),u_smc_nou(1:id,2),'r', ...
     time(1:id),u_fbl_nou(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

saveas(gcf,'nomass_u.fig','fig')

% error
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),e_dar_nom(1:id,1),'b',time(1:id),e_dar_nom(1:id,2),'b--', ...
     time(1:id),e_smc_nom(1:id,1),'r',time(1:id),e_smc_nom(1:id,2),'r--', ...
     time(1:id),e_fbl_nom(1:id,1),'g',time(1:id),e_fbl_nom(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3])
hold off;

nexttile
hold on; grid on;
plot(time(1:id),e_dar_nou(1:id,1),'b',time(1:id),e_dar_nou(1:id,2),'b--', ...
     time(1:id),e_smc_nou(1:id,1),'r',time(1:id),e_smc_nou(1:id,2),'r--', ...
     time(1:id),e_fbl_nou(1:id,1),'g',time(1:id),e_fbl_nou(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3])
hold off;

lg = legend("$DAR-z_{\theta 1}$","$DAR-z_{\theta 2}$", ...
       "$SMC-z_{\theta 1}$","$SMC-z_{\theta 2}$", ...
       "$FBL-z_{\theta 1}$","$FBL-z_{\theta 2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'nomass_e.fig','fig')

%% 2 Kg mass 

% position
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),q_dar_2m(1:id,2),'b',time(1:id),q_dar_2m(1:id,4),'b--', ...
     time(1:id),q_smc_2m(1:id,2),'r',time(1:id),q_smc_2m(1:id,4),'r--', ...
     time(1:id),q_fbl_2m(1:id,2),'g',time(1:id),q_fbl_2m(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

nexttile
hold on; grid on;
plot(time(1:id),q_dar_2u(1:id,2),'b',time(1:id),q_dar_2u(1:id,4),'b--', ...
     time(1:id),q_smc_2u(1:id,2),'r',time(1:id),q_smc_2u(1:id,4),'r--', ...
     time(1:id),q_fbl_2u(1:id,2),'g',time(1:id),q_fbl_2u(1:id,4),'g--', ...
     time(1:id),ref(1,1:id),'k-.',time(1:id),ref(2,1:id),'k:','linewidth',2)
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$x_{\theta}[rad]$",'interpreter','latex')
hold off;

lg = legend("$DAR-x_{\theta 1}$","$DAR-x_{\theta 2}$", ...
       "$SMC-x_{\theta 1}$","$SMC-x_{\theta 2}$", ...
       "$FBL-x_{\theta 1}$","$FBL-x_{\theta 2}$", ...
       "$REF-\omega_{\theta_1}$","$REF-\omega_{\theta_2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'2kg_pos.fig','fig')

% control
figure()
tiledlayout(4,4)

nexttile(1,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_2m(1:id,1),'b',time(1:id),u_dar_2m(1:id,2),'b--', ...
     time(1:id),u_smc_2m(1:id,1),'r',time(1:id),u_smc_2m(1:id,2),'r--', ...
     time(1:id),u_fbl_2m(1:id,1),'g',time(1:id),u_fbl_2m(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

lg = legend("$DAR-u_1$","$DAR-u_2$", ...
       "$SMC-u_1$","$SMC-u_2$", ...
       "$FBL-u_1$","$FBL-u_2$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

nexttile(3,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_2m(1:id,1),'b', ...
     time(1:id),u_smc_2m(1:id,1),'r', ...
     time(1:id),u_fbl_2m(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(7,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_2m(1:id,2),'b', ...
     time(1:id),u_smc_2m(1:id,2),'r', ...
     time(1:id),u_fbl_2m(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

nexttile(9,[2,2])
hold on; grid on;
plot(time(1:id),u_dar_2u(1:id,1),'b',time(1:id),u_dar_2u(1:id,2),'b--', ...
     time(1:id),u_smc_2u(1:id,1),'r',time(1:id),u_smc_2u(1:id,2),'r--', ...
     time(1:id),u_fbl_2u(1:id,1),'g',time(1:id),u_fbl_2u(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u[N \cdot m]$",'interpreter','latex')
xlim([0,80]);
ylim([-110,110]);
xticks([0,10,20,30,40,50,60,70,80])
hold off;

nexttile(11,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_2u(1:id,1),'b', ...
     time(1:id),u_smc_2u(1:id,1),'r', ...
     time(1:id),u_fbl_2u(1:id,1),'g')
%xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_1[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([0,50]);
xticks([])
yticks([0,25,50])
hold off;

nexttile(15,[1,2])
hold on; grid on;
plot(time(1:id),u_dar_2u(1:id,2),'b', ...
     time(1:id),u_smc_2u(1:id,2),'r', ...
     time(1:id),u_fbl_2u(1:id,2),'g')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$u_2[N \cdot m]$",'interpreter','latex')
xlim([100,300]);
ylim([-10,20]);
hold off;

saveas(gcf,'2kg_u.fig','fig')

% error
figure()
tiledlayout(2,1)

nexttile
hold on; grid on;
plot(time(1:id),e_dar_2m(1:id,1),'b',time(1:id),e_dar_2m(1:id,2),'b--', ...
     time(1:id),e_smc_2m(1:id,1),'r',time(1:id),e_smc_2m(1:id,2),'r--', ...
     time(1:id),e_fbl_2m(1:id,1),'g',time(1:id),e_fbl_2m(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3])
hold off;

nexttile
hold on; grid on;
plot(time(1:id),e_dar_2u(1:id,1),'b',time(1:id),e_dar_2u(1:id,2),'b--', ...
     time(1:id),e_smc_2u(1:id,1),'r',time(1:id),e_smc_2u(1:id,2),'r--', ...
     time(1:id),e_fbl_2u(1:id,1),'g',time(1:id),e_fbl_2u(1:id,2),'g--')
xlabel("$Time[s]$",'interpreter','latex')
ylabel("$z_{\theta}[rad]$",'interpreter','latex')
ylim([-0.3,0.3])
hold off;

lg = legend("$DAR-z_{\theta 1}$","$DAR-z_{\theta 2}$", ...
       "$SMC-z_{\theta 1}$","$SMC-z_{\theta 2}$", ...
       "$FBL-z_{\theta 1}$","$FBL-z_{\theta 2}$", ...
       'Location','se','NumColumns',4,'interpreter','latex');
lg.Layout.Tile = 'north';

saveas(gcf,'2kg_e.fig','fig')

