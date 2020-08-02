# Designing-an-Overcurrent-CO-8-Relay
The project involved designing an inverse overcurrent (CO-8) relay with user selectable tap and time dial settings. The relay recorded the current level, time stamp of relay tripping and reset. The mathematical model was implemented using MATLAB. 
ABSTRACT: This project involves the use of standard electromechanical Over current relay (CO‐8). The relay operation is tested in this project under various operating conditions. The results obtained by running the code must satisfy the time curve of over current relay (CO-8). The theoretical mathematical model which is to be used to design the overcurrent relay is implemented in the software MATLAB. In order to filter out the harmonics and DC offset, cosine filter is used instead of Low pass FIR filter as it does not clear Dc offset. In order to make the operation of the electro mechanical relay clear, faults currents of one level and two levels are used. In this project three tap settings and three time dial settings are used to check the operation of the relay.

This Project was created to as a part of coursework for UTA EE 5374 under Dr Wei-Jen Lee.
HOW TO RUN THE CODE: 1) Make sure the Excel files (attached in the repository) are in the same folder as the .m file.

2) Apply the following Code

%% Defining initial conditions
info=xlsread('Check Signal1.xlsx');
time = xlsread('TIME.xlsx');
s1= info(:,1);
s2= info(:,2);
s3= info(:,3);
t = time(:,1);
Ts= input('enter your tap setting ');
td= input('enter your time dial setting ');
 
freqs=16; 
freq=60; 
 
%% Finding Amplitudes, RMS and Multipliers
 
u = length(s1);
 
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
M1 = RMS1/Ts;
M2 = RMS2/Ts;
M3 = RMS3/Ts;
 
%% Recording Faults and sending Fault and reset signals 
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
 o = M3(i);
    if o>4
        disp('Fault Detected for M3 at')
        find o;
        disp(o)
    end
end
 
 
for i = 1:u-1
 m = M3(i);
    if m<4
        disp('System reset for M3 at')
        find m;
        disp(m)
    end
end
 
%% Applying cosine filtering and calculating tripping time
for i=1:freqs
rcos(i)=cos(2*pi*(i-1)/16); 
end
 
for j=2:u-1 
m=1;
for i=1:freqs
I(i)=info(j+15-m)*rcos(m); 
m=m+1;
end
If(j)=2*sum(I)/16; 
end
 
for i=1:u-4688 
 
if flag==1 
break;
end
for j=1:16
    
RMSI = ((If(((i)*16)-j+1))^2)^0.5;
end
 
M=RMSI/Ts;
 
 
triptime=td*((5.95/(M^2-1))+0.18); % calculating secs. for disc rotation
 
 
end 
 
disp('The tripping time for the given Tap and time dial settings is')
disp(triptime);
 
 
%% Plotting the required Graphs
 
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
t1 = time(:,2);
M3d = (If)';
M32 = M3d/sqrt(2);
M3f = M32/Ts;
triptime1=td*((5.95/(M3f.^2-1))+0.18);
olk = triptime1';
 
figure
plot(t,M3f)
xlabel('time')
ylabel('Multiplier')
title('Filtered signal of three current levels with DC offset')
figure
plot(olk,t)
ylabel('samples')
xlabel('tripping time')
title('Tripping time for filtered signal with given tap and time dial settings')


