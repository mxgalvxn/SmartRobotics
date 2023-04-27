time = (0:0.01:6)';
x = sin(40./(time+0.01));
figure()
plot(time,x)
title('Information Signal x','fontsize',10)
xlabel('time','fontsize',10)
ylabel('x','fontsize',10)

n1 = randn(size(time));

figure()
plot(time,n1)
title('Noise Source n_1','fontsize',10)
xlabel('time','fontsize',10)
ylabel('n_1','fontsize',10)

domain = linspace(min(n1),max(n1),20);
[xx,yy] = meshgrid(domain,domain);
zz = 4*sin(xx).*yy./(1+yy.^2);
surf(xx,yy,zz);
xlabel('n_1(k)','fontsize',10);
ylabel('n_1(k-1)','fontsize',10);
zlabel('n_2(k)','fontsize',10);
title('Unknown Interference Channel Characteristics','fontsize',10);
n1d0 = n1; % n1 with delay 0
n1d1 = [0; n1d0(1:length(n1d0)-1)]; % n1 with delay 1
n2 = 4*sin(n1d0).*n1d1./(1+n1d1.^2); % interference
subplot(2,1,1)
plot(time,n1);
ylabel('noise n_1','fontsize',10);
subplot(2,1,2)
plot(time,n2);
ylabel('interference n_2','fontsize',10);

m = x + n2; % measured signal
subplot(1,1,1)
figure()
plot(time, m)
title('Measured Signal','fontsize',10)
xlabel('time','fontsize',10)
ylabel('m','fontsize',10)

delayed_n1 = [0; n1(1:length(n1)-1)];
data = [delayed_n1 n1 m];

genOpt = genfisOptions('GridPartition');
inFIS = genfis(data(:,1:end-1),data(:,end),genOpt);
trainOpt = anfisOptions('InitialFIS',inFIS,'InitialStepSize',0.2);
outFIS = anfis(data,trainOpt);

estimated_n2 = evalfis(data(:,1:2),outFIS);
estimated_x = m - estimated_n2;

figure()
subplot(2,1,1)
plot(time, n2)
ylabel('n_2 (unknown)');
subplot(2,1,2)
plot(time, estimated_n2)
ylabel('Estimated n_2')
% 
figure
plot(time,estimated_x,'b',time,x,'r')
legend('Estimated x','Actual x (unknown)','Location','SouthEast')
