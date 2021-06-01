function []=test_benchmark_traj(G,K,time_steps,delta_t)
[So,Si,Gp]=get_loop_tfs(G,K);
ny=size(So,1);
time=delta_t*(0:(time_steps-1));
%%
r=zeros(ny, time_steps);
% Benchmark trajectory
r(1,:)=4*sin(6*pi/100*time);
r(2,:)=8*sin(6*pi/200*time);
r(3,:)=0.1*time;
%% Time domain simulation
y=lsim(eye(ny)-So,r,time,'r');
%%
figure()
plot3(r(1,:),r(2,:),r(3,:))
hold on
plot3(y(:,1),y(:,2),y(:,3))
legend('reference','output')
title('Response for Reference trajectory')

figure()
lsim(K*So,r,time,'r')
title('control inputs')
end