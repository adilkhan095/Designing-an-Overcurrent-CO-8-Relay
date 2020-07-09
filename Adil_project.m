info= xlsread('Check Signal.xlsx');
t= info(:,1);
s1= info(:,2);
s2= info(:,3);
s3= info(:,4);
ts= 3.5;
tds= 2;

% tds1 = tds/7;
f= 60;
%Finding Amplitude
u = length(s1);
% Finding Amplitude
% for i = 1:(u-1)
% qw = ((s1(i).^2) - (s1(i).^2 - 4));
% A1(i,:) = qw;
% rw = ((s2(i).^2) - (s2(i).^2 - 4));
% A2(i,:) = rw;
% sw = ((s3(i).^2) - (s3(i).^2 - 4));
% A3(i,:) = sw;
% end



for i = 1:(u-1)
    m = s1(i,1);
    n = s1(i+1,1);
    qw =sqrt(((m.^2)+(n.^2)-2*m*n*cos(0.3935)))/(sin(0.3925));
    A1(i,:)= qw;
    
end
clear i m n 
for i = 1:(u-1)
    m = s2(i,1);
    n = s2(i+1,1);
    rw =sqrt(((m.^2)+(n.^2)-2*m*n*cos(0.3935)))/(sin(0.3925));
    A2(i,:)= rw;
end
% clear i m n 
for i = 1:(u-1)
    m = s3(i,1);
    n = s3(i+1,1);
    sw =sqrt(((m.^2)+(n.^2)-2*m*n*cos(0.3935)))/(sin(0.3925));
    A3(i,:)= sw;
end
clear i m n
%Now finding the RMS current values
RMS1 = A1/sqrt(2);
RMS2 = A2/sqrt(2);
RMS3 = A3/sqrt(2);

%finding the multipliers
M1 = RMS1/ts;
M2 = RMS2/ts;
M3 = RMS3/ts;
N = 4999;
f0 = 0.00104;


filtr = firceqrip(4999,0.104,[0.00100 0.10]);
M3f = filter(filtr,1,M3);
M3fd = flipud(M3f);

%displaying fault for M1 and resetting it
for i = 1:u-1
 o = M1(i);
    if o>3.5
        disp('Fault Detected for M1 at')
        find o;
        disp(o)
    end
end

for i = 1:u-1
    m = M1(i);
if m<3.5
    disp('System reset for M1 at')
     find m;
    disp(m)
end
end
clear i m o
%displaying fault for M2 and resetting it

for i = 1:u-1
 o = M2(i);
    if o>4
        disp('Fault Detected for M2 at')
        find o;
        disp(o)
    end
end

for i = 1:u-1
    m = M1(i);
if m<4
    disp('System reset for M2 at')
     find m;
    disp(m)
end
end

clear i m o

%displaying fault for M3 and resetting it

for i = 1:u-1
 o = M3fd(i);
    if o>4
        disp('Fault Detected for M3 at')
        find o;
        disp(o)
    end
end


for i = 1:u-1
 m = M3fd(i);
    if m<4
        disp('System reset for M3 at')
        find m;
        disp(m)
    end
end

%calculating time delay, We got our first fault at 959
% freq is 60hz and we have 16 samples/cycle
timedelay = 959*(0.0166/16);

% Applying the Multiplier to find the relay tripping time 
for i = 1:u-1
    n = M1(i);
m = tds*((5.95/(n.^2-1))+0.18);
td1(i,1) = m;
end
clear i m n


for i = 1:u-1
n = M2(i);    
m = tds*((5.95/(n.^2))+0.18);
td2(i,1) = m;
end
clear i m n

for i = 1:u-1
n = M3fd(i);    
m = tds*((5.95/(n.^2-1))+0.18);
td3(i,1) = m;
end

%Calculating the final relay tripping time

rtt1 = td1 + timedelay;
rtt2 = td2 + timedelay;
rtt3 = td3 + timedelay;

figure
plot(t(960:4999,1),td2(960:4999,1))
xlabel('samples')
ylabel('tripping time')
title('Tripping time graph for 2nd signal')



%Plotting t vs M1 graph
figure
plot(t(1:4999,1),M1)
xlabel('time')
ylabel('Multiplier')
title('Two current levels')
%Plotting t vs M2 graph
figure
plot(t(1:4999,1),M2)
xlabel('time')
ylabel('Multiplier')
title('Three current levels')
%Plotting t vs M3 graph
figure
plot(t(1:4999,1),M3)
xlabel('time')
ylabel('Multiplier')
title('Three current levels with DC offset')

%applying filter on the third signal which has Dc offsets

figure
plot(t(1:4999,1),M3fd)
xlabel('time')
ylabel('Multiplier')
title('Filtered signal')



% %% Plotting the intital conditions
% 
% for i=1:length(s1)
%     
% rtt1= 2*((5.95/(s1(i).^2-1))+0.18)';
% qw(i,1)=rtt1;
% 
% rtt2= 2*((5.95/(s1(i).^2-1))+0.18)';
% rw(i,1)=rtt2;
% 
% rtt3= 2*((5.95/(s1(i).^2-1))+0.18)';
% sw(i,1)=rtt3;
% 
% figure
%  plot3(t,qw(1:4999),s1(1:4999),'-r')
%     axis([-25 25 -10 5000 -13 5])
%     title('Pure sinusoidal with two current levels')
%     xlabel(' given signal s1')
%     ylabel('time')
%     zlabel('tripping time')
% hold on
%        plot3(t,rw(1:4999),s2(1:4999),'-r')
%        axis([-25 25 -20 5000 -15 5])
%        title('Pure sinusoidal with four current levels')
%        xlabel(' given signal s2')
%        ylabel('time')
%        zlabel('tripping time')
%  hold on
%     plot3(t,sw(1:4999),s3(1:4999), '-k')
%     axis([-50 50 -10 5000 -13 5])
%     title('sinusoidal with four current levels and DC offset')
%     xlabel(' given signal s3')
%     ylabel('time')
%     zlabel('relay tripping time')
% end
%     clear rtt i qw rw sw


