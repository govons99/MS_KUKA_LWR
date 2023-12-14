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

ref = [0.4*cos(t);-0.4*cos(t)];

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
ylim([-0.7,0.7])
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

%% control

K = [-9277.13944860092 3710.17708672289 -349.234832428913 266.395117894730 -194809.308523000 29590.8387072452 -497382.571831891 83024.4056398725 -512598.396322723 96517.0532040094 -268819.441373604 59336.0150773882 -73589.1199037631 20303.8593336261;
-2029.21453824429 -7028.36935443040 -80.0713649245077 -523.663369755656 -42740.8155134585 -58507.6257749021 -108901.648552769 -163109.287869734 -111981.948671448 -188381.316026516 -58595.8106911847 -115090.430611396 -16021.4507995792 -39186.9823159717];

for i = 1:max(size(eta))
    control(:,i) = K * [q(i,2);q(i,4);dq(i,2);dq(i,4);eta(i,:)'];
    if ( abs(control(1,i)) >= 1000 )
	control(1,i) = sign(control(1,i))*1000;
    end
    if ( abs(control(2,i)) >= 1000 )
	control(2,i) = sign(control(2,i))*1000;
    end
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


