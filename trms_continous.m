%% LQR in Continous

A = [
         0         0    1.0000         0;
         0         0         0    1.0000;
   -0.8245         0   -0.2933         0;
   -0.0300         0         0   -1.2850];


B = [    0         0;
         0         0;
    0.1480e-3   -0.0009e-3;
   -0.0049e-3    0.0276e-3];

C = [1 0 0 0; 0 1 0 0];

D = zeros(2);


Q = [1e8 0 0 0;
     0 1e8 0 0;
     0 0 1 0;
     0 0 0 1];
R = [1 0; 0 1];

[K, S, e] = lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [-Ac];
Cc = [C];
Dc = zeros(2,4);

sys_cl = ss(Ac,Bc,Cc,Dc);

tFinal = 100;
time_total = 0:0.01:tFinal;

Ns = size(time_total);
Ns = Ns(2);

xd = [0.2;-0.05;0;0];
closed_loop_input=[xd(1)*ones(size(time_total));
                   xd(2)*ones(size(time_total));
                   xd(3)*ones(size(time_total));
                   xd(4)*ones(size(time_total))];
closed_loop_input(1,2000:end) = 0.3;
closed_loop_input(2,2000:end) = -0.05;


uk = zeros(2,Ns);
ud = zeros(2,Ns);

for i=1:Ns
    ud(:,i) = -inv(B'*B)*B'*A*closed_loop_input(:,i);
end

[output_closed_loop,time_closed_loop,state_closed_loop] = lsim(sys_cl,closed_loop_input,time_total);

state_closed_loop = state_closed_loop';
for i=1:Ns
   uk(:,i) = ud(:,i) - K*(state_closed_loop(:,i) - xd);
end

figure(1)
subplot(4,1,1),plot(time_total,state_closed_loop(1,:),'b-'),grid,ylabel("X1"),title('State Variables')
subplot(4,1,2),plot(time_total,state_closed_loop(2,:),'b-'),grid,ylabel("X2")
subplot(4,1,3),plot(time_total,state_closed_loop(3,:),'b-'),grid,ylabel("X3")
subplot(4,1,4),plot(time_total,state_closed_loop(4,:),'b-'),grid,ylabel("X4"),xlabel("Time (sec)")

figure(2)
subplot(2,1,1),plot(time_total,closed_loop_input(1,:),'b-'),grid,ylabel("U1"),title('Inputs')
subplot(2,1,2),plot(time_total,closed_loop_input(2,:),'b-'),grid,ylabel("U2")

figure(3)
subplot(2,1,1),plot(time_total,uk(1,:),'b-'),grid,ylabel("U1"),title('Inputs')
subplot(2,1,2),plot(time_total,uk(2,:),'b-'),grid,ylabel("U2")




% Now simulate the discrete system

dt_angles = 0.01;
sys_cont = ss(A,B,C,D);
sys_disc = c2d(sys_cont, dt_angles);



xk_disc = zeros(4,Ns);
uk_disc = zeros(2,Ns);

%us = zeros(2,Ns);
%xs = zeros(4,Ns);

for k=1:Ns-1

    xk_disc(:,k+1) = sys_disc.A*xk_disc(:,k) + sys_disc.B*uk(:,k);

end

figure(4)   % For Myself
subplot(4,1,1),plot(time_total,xk_disc(1,:),'b-'),grid,ylabel("X1"),title('State Variables')
subplot(4,1,2),plot(time_total,xk_disc(2,:),'b-'),grid,ylabel("X2")
subplot(4,1,3),plot(time_total,xk_disc(3,:),'b-'),grid,ylabel("X3")
subplot(4,1,4),plot(time_total,xk_disc(4,:),'b-'),grid,ylabel("X4"),xlabel("Time (sec)")

%% Code 1
%t = 0:0.01:100;
%samps = size(t);
%samp_s = samps(2);
%r = [1228.791 229.683] + zeros(samp_s,2);
%r(:,1) = r(:,1) + 0.3;


%[y,t,x] = lsim(sys_cl,r,t);

%figure(1)   % For Myself
%subplot(4,1,1),plot(t,x(:,1),'b-'),grid,ylabel("X1"),title('State Variables')
%subplot(4,1,2),plot(t,x(:,2),'b-'),grid,ylabel("X2")
%subplot(4,1,3),plot(t,x(:,3),'b-'),grid,ylabel("X3")
%subplot(4,1,4),plot(t,x(:,4),'b-'),grid,ylabel("X4"),xlabel("Time (sec)")

%figure(2)
%subplot(2,1,1),plot(t,r(:,1),'b-'),grid,ylabel("U1"),title('Inputs')
%subplot(2,1,2),plot(t,r(:,2),'b-'),grid,ylabel("U2")





%% Code 2

%states = {'pitch' 'yaw' 'pitch_dot' 'yaw_dot'};
%inputs = {'wm' 'wt'};
%outputs = {'pitch'; 'yaw'};

%sys_ol = ss(A,B,C,D);
%xd = [0.7;0.05;0;0];
%x0 = [0;0;0;0];

%ud = -inv(B'*B)*B'*A*xd;

%tFinal = 30;
%time_total = 0:0.01:tFinal;

%input_ud=ud*ones(size(time_total));

%[output_ud_only,time_ud_only,state_ud_only] = lsim(sys_ol,input_ud,time_total);

%figure(1)
%subplot(4,1,1),plot(time_total,state_ud_only(:,1),'b-'),grid,ylabel("X1"),title('State Variables')
%subplot(4,1,2),plot(time_total,state_ud_only(:,2),'b-'),grid,ylabel("X2")
%subplot(4,1,3),plot(time_total,state_ud_only(:,3),'b-'),grid,ylabel("X3")
%subplot(4,1,4),plot(time_total,state_ud_only(:,4),'b-'),grid,ylabel("X4"),xlabel("Time (sec)")

%figure(2)
%subplot(2,1,1),plot(time_total,input_ud(1,:),'b-'),grid,ylabel("U1"),title('Inputs')
%subplot(2,1,2),plot(time_total,input_ud(2,:),'b-'),grid,ylabel("U2")


















