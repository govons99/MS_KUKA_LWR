clc
clear
close all

q = load("Q.txt");

dq = load("dQ.txt");
dq_hat = load("dQ_hat.txt");

%% Joint position

figure(1)
hold on; grid on;
for i=1:7
    hold on;
    plot(q(:,i));
end
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





