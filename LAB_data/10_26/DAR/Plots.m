K = [-93.1726   -8.4440  -21.5335   -4.7909;
     -17.7032  -34.4521   -6.6227  -20.5367];
 
 
Q = load("Q.txt");
dQ = load("dQ.txt");

dim = max(size(Q));

u = zeros(2,dim);

for i = 1:dim
    u(:,i) = K*[Q(i,1);Q(i,4);dQ(i,1);dQ(i,4)];
end

figure()
plot(u(1,:))