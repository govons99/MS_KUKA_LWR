clc
clear
close all

q = load("Q.txt");

dq_hat = load("dQ_hat.txt");
dq = load("dQ.txt");

eta = load("eta.txt");

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

a01 = 0.1444;
a11 = -0.4403;
b11 = -0.2636;
f1 = 5;

a02 = 0.9537;
a12 = 0.3598;
b12 = -0.2501;
f2 = 5;

w1 = a01 + a11*cos(t*f1) + b11*sin(t*f1);
w2 = a02 + a12*cos(t*f2) + b12*sin(t*f2);

ref = [w1;w2];

figure(6)
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
plot(time(1,100000:end-1),error(100000:end,1),'b','linewidth',1.5);
plot(time(1,100000:end-1),error(100000:end,2),'r','linewidth',1.5);

%% Joint position

figure(1)
hold on; grid on;
plot(time,q(:,2),'b','linewidth',2)
plot(time,q(:,4),'r','linewidth',2)
plot(time,ref(1,1:dim),'--b','linewidth',2)
plot(time,ref(2,1:dim),'--r','linewidth',2)
%title('joint position');
xlabel("t [s]")
ylabel("\theta [rad]")
legend('\theta_1','\theta_2','\theta_1^*','\theta_2^*','NumColumns',2);

%% joint speed

figure(2)
hold on; grid on;
plot(time,dq_hat(:,2));
plot(time,dq_hat(:,4));
title('joint speed (estimated)');
legend('dq1','dq2');

%% internal model

figure()
hold on; grid on;
plot(time(1:end-1),eta,'linewidth',2)
xlabel("t [s]")
ylabel("\eta [rad]")
%title('internal model')

%% ee position

x = 0.5*cos(q(:,2))+0.5*cos(q(:,2)+q(:,4));
y = 0.5*sin(q(:,2))+0.5*sin(q(:,2)+q(:,4));

x_ref = 0.5*cos(w1)+0.5*cos(w1+w2);
y_ref = 0.5*sin(w1)+0.5*sin(w1+w2);

figure()
hold on; grid on;
plot(x,y,'LineWidth',2)
plot(x_ref,y_ref,'LineWidth',2)
legend('EE','EE reference')
xlabel("x [m]")
ylabel("y [m]")
title("KUKA Dumpling")

%% control

K = [-51479.1506564270 3568.82244932025 -11854.7020446130 1079.87266377154 -633338.094409722 4333.26737636959 508505.390501465 -47862.5287158016 -98358.8518500649 6135.56008409700 ;
1263.68859842101 -44137.3005591795 480.698288000380 -10896.5985110252 -9654.94397920896 -440771.759801160 -20520.6564484375 470082.073219219 1813.31207683121 -82035.0166304382 ];

for i = 1:max(size(eta))
    control(:,i) = K * [q(i,2);q(i,4);dq(i,2);dq(i,4);eta(i,:)'];
    % if ( abs(control(1,i)) >= 15000 )
	% control(1,i) = sign(control(1,i))*15000;
    % end
    % if ( abs(control(2,i)) >= 15000 )
	% control(2,i) = sign(control(2,i))*15000;
    % end
end

figure()
hold on; grid on;
plot(time(1:end-1),control','linewidth',2)
xlabel("t [s]")
ylabel("u [Nm]")
legend("u_1","u_2")
%title('control')

axes('Position',[.2 .75 .15 .15])
box on;
hold on; grid on;
plot(time(1,1:4000),control(1,1:4000),'b','linewidth',1.5);
plot(time(1,1:4000),control(2,1:4000),'r','linewidth',1.5);

