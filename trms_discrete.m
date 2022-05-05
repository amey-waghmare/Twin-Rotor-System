%% LQR

A = [1           0    0.009985           0;
   -1.503e-06           1  -5.012e-09    0.009936;
   -0.008234           0       0.997           0;
   -0.0003           0  -1.502e-06      0.9872];

B = [7.397e-09  -4.615e-11;
   -2.461e-10   1.378e-09;
   1.479e-06  -9.225e-09;
   -4.911e-08    2.75e-07];

C = [1   0   0   0;
   0   1   0   0];

dt_angles = 0.01;

Q = [7e13 0 0 0;
     0 2e15 0 0;
     0 0 9e12 0;
     0 0 0 10];
R = [9e3 0; 0 5e4];

us = [1228.791; 229.683];


%xs = [0.2;0;0;0];

[K, S, e] = dlqr(A,B,Q,R);

Ns = 3000;


xk = zeros(4,Ns);
uk = zeros(2,Ns);

%us = zeros(2,Ns);
%xs = zeros(4,Ns);

kT = zeros(1,Ns);        % Time Array

for k=1:Ns-1

    if (k <= 1000)
        xs = [0.3;-0.05;0;0];
    elseif (k > 1000)
        xs = [0.1;0.0;0;0];
    end


    kT(k) = (k-1)*dt_angles;

    uk(:,k) = us - K*(xk(:,k) - xs);
    xk(:,k+1) = A*xk(:,k) + B*uk(:,k);

end
kT(Ns) = (Ns-1)*dt_angles;

figure(1)   % For Myself
subplot(4,1,1),plot(kT,xk(1,:),'b-'),grid,ylabel("X1"),title('State Variables')
subplot(4,1,2),plot(kT,xk(2,:),'b-'),grid,ylabel("X2")
subplot(4,1,3),plot(kT,xk(3,:),'b-'),grid,ylabel("X3")
subplot(4,1,4),plot(kT,xk(4,:),'b-'),grid,ylabel("X4"),xlabel("Time (sec)")

figure(2)
subplot(2,1,1),plot(kT,uk(1,:),'b-'),grid,ylabel("U1"),title('Inputs')
subplot(2,1,2),plot(kT,uk(2,:),'b-'),grid,ylabel("U2")


%% My Implementation
clear;
clc;
load("discrete.mat")

%A = [1           0    0.009985           0;
%   -1.503e-06           1  -5.012e-09    0.009936;
%   -0.008234           0       0.997           0;
%   -0.0003           0  -1.502e-06      0.9872];

%B = [7.397e-09  -4.615e-11;
%   -2.461e-10   1.378e-09;
%   1.479e-06  -9.225e-09;
%   -4.911e-08    2.75e-07];

%C = [1   0   0   0;
%   0   1   0   0];

dt_angles = 0.01;

Q = [7e11 0 0 0;
     0 2e13 0 0;
     0 0 1e2 0;
     0 0 0 1e2];
R = [9e3 0; 0 5e4];


[K, S, e] = dlqr(sys_dt.A,sys_dt.B,Q,R);

Ns = 4000;


xk = zeros(4,Ns);
uk = zeros(2,Ns);

us = zeros(2,Ns);
%xs = zeros(4,Ns);

kT = zeros(1,Ns);        % Time Array

for k=1:Ns-1

    if (k <= 2000)
        xd = [0.3;-0.05;0;0];
    elseif (k > 2000)
        xd = [0.2;0.0;0;0];
    end

    us = inv(sys_dt.B'*sys_dt.B)*sys_dt.B'*(eye(4)- sys_dt.A)*xd;


    kT(k) = (k-1)*dt_angles;

    uk(:,k) = us - K*(xk(:,k) - xd);
    xk(:,k+1) = sys_dt.A*xk(:,k) + sys_dt.B*uk(:,k);

end
kT(Ns) = (Ns-1)*dt_angles;

figure(3)   % For Myself
subplot(4,1,1),plot(kT,xk(1,:),'b-'),grid,ylabel("pitch"),title('State Variables')
subplot(4,1,2),plot(kT,xk(2,:),'b-'),grid,ylabel("yaw")
subplot(4,1,3),plot(kT,xk(3,:),'b-'),grid,ylabel("pitch rate")
subplot(4,1,4),plot(kT,xk(4,:),'b-'),grid,ylabel("yaw rate"),xlabel("Time (sec)")

figure(4)
subplot(2,1,1),plot(kT,uk(1,:),'b-'),grid,ylabel("U1"),title('Inputs')
subplot(2,1,2),plot(kT,uk(2,:),'b-'),grid,ylabel("U2")

