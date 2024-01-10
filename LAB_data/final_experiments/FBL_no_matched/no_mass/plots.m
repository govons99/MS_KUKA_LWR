clc
clear
close all

q = load("Q.txt");
q_hat = load("Q_hat.txt");

dq_hat = load("dQ_hat.txt");
dq = load("dQ.txt");

%eta = load("eta.txt");

q_ref = load("Qref.txt");

error = load("error.txt");

control = load("control.txt");

control_DAR = load("control_DAR.txt");

%% reference

t = 0:0.0001:600;

%ref = [q(1,2) + 0.1*(1-cos(t));q(1,4) + 0.1*(1-cos(t))];
ref = [0.4*cos(t);-0.4*cos(t)];

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
%ref = [w1;w2];

figure()
hold on; grid on;
plot(error(:,1))
plot(error(:,2))
title('error')

%% Joint position

figure()
hold on; grid on;
plot(q(:,2),'b','linewidth',2.5)
plot(q(:,4),'r','linewidth',2.5)
plot(ref(1,1:max(size(q))),'--b','linewidth',2.5)
plot(ref(2,1:max(size(q))),'--r','linewidth',2.5)
title('joint position');
legend('q1','q2','qref1','qref2');

%% joint speed

figure()
hold on; grid on;
plot(dq_hat(:,2));
plot(dq_hat(:,4));
title('joint speed (estimated)');
legend('dq1','dq2');

figure()
hold on; grid on;
plot(dq(:,2),'--');
plot(dq(:,4),'--');

%% control

figure()
hold on; grid on;
plot(control_DAR(:,1),'b');
plot(control_DAR(:,2),'r');

%% internal model

K = [-9277.13944860092 3710.17708672289 -349.234832428913 266.395117894730 -194809.308523000 29590.8387072452 -497382.571831891 83024.4056398725 -512598.396322723 96517.0532040094 -268819.441373604 59336.0150773882 -73589.1199037631 20303.8593336261;
-2029.21453824429 -7028.36935443040 -80.0713649245077 -523.663369755656 -42740.8155134585 -58507.6257749021 -108901.648552769 -163109.287869734 -111981.948671448 -188381.316026516 -58595.8106911847 -115090.430611396 -16021.4507995792 -39186.9823159717];

for i = 1:max(size(eta))-1
    control(:,i) = K * [q(i,2);q(i,4);dq(i,2);dq(i,4);eta(i,:)'];
    if ( abs(control(1,i)) >= 1000 )
	control(1,i) = sign(control(1,i))*1000;
    endif
    if ( abs(control(2,i)) >= 1000 )
	control(2,i) = sign(control(2,i))*1000;
    endif
end

figure()
hold on; grid on;
plot(eta)
title('internal model')

figure()
hold on; grid on;
plot(control')
title('control')


