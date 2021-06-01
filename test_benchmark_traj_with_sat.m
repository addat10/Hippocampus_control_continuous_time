function []=test_benchmark_traj_with_sat(G,K,time_steps,delta_t)
[So,~,~]=get_loop_tfs(G,K);
ny=size(So,1);
time=delta_t*(0:(time_steps-1));
%%
r=zeros(ny, time_steps);
% Benchmark trajectory
r(1,:)=4*sin(6*pi/100*time);
r(2,:)=8*sin(6*pi/200*time);
r(3,:)=0.1*time;
%% Time domain simulation
% get sizes
nx_p=size(G.A,1); ny=size(G.C,1);
nx_c=size(K.A,1); nu=size(K.C,1);

% initialize variables
x_p=zeros(nx_p,time_steps);
x_c=zeros(nx_c,time_steps);
u=zeros(nu,time_steps);
y=zeros(ny,time_steps);
e=zeros(ny,time_steps);

for i=1:(time_steps-1)    
    e(:,i)=r(:,i)-y(:,i);    
    % Update controller states
    [x_c(:,i+1)]=propagate_state(K,delta_t,x_c(:,i),e(:,i));
    u(:,i)=K.C*x_c(:,i)+K.D*e(:,i);
    % Saturate input u
    u(:,i)=saturate_u(u(:,i));
    % Update plant states
    [x_p(:,i+1)]=propagate_state(G,delta_t,x_p(:,i),u(:,i));
    y(:,i+1)=G.C*x_p(:,i+1);
end
%y=lsim(eye(ny)-So,r,time,'r');
%%
figure()
plot3(r(1,1:(end-1)),r(2,1:(end-1)),r(3,1:(end-1)))
hold on
plot3(y(1,1:(end-1)),y(2,1:(end-1)),y(3,1:(end-1)),'ro')
legend('reference','output')
title('Response for Reference trajectory')

figure()
plot(time,u)
legend('control input')
title('control inputs')
end