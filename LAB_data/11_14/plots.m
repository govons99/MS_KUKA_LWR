clc
clear
close all

q = load("Q.txt");

dq = load("dQ.txt");
dq_hat = load("dQ_hat.txt");

dim = max(size(q));
index = 0.0;
time = zeros(1,dim);

for i=1:dim
    time(i) = index;
    index = index+0.005;
end

for i = 1:dim
    q_ref(:,i) = q(1,:);
    q_ref(2,i) = 0.4*cos(0.5*time(i));
    q_ref(4,i) = -0.4*cos(0.5*time(i));
end

%% Joint position

figure(1)
hold on; grid on;
    plot(time,q(:,2),'b');
    plot(time,q(:,4),'r'); 
    plot(time,q_ref(2,:),'b--');
    plot(time,q_ref(4,:),'r--');

title('joint position');
legend('1','2','3','4','5','6','7');

%% joint speed

figure(2)
hold on; grid on;
for i=1:7
    hold on;
    plot(dq(:,i));
end
title('joint speed');
legend('1','2','3','4','5','6','7');

figure()
hold on; grid on;
for i=1:7
    hold on;
    plot(dq_hat(:,i));
end
title('joint speed');
legend('1','2','3','4','5','6','7');





