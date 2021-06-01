function []=time_domain_analysis(G,K)
%% Simulation parameters
meas_noise=0;
input_dist=1;
time_steps=10000;
delta_t=0.001;

time=delta_t*(0:(time_steps-1));

% Get sizes
nx=size(G.A,1);
nu=size(G.B,2);
ny=size(G.C,1);

% Reference and disturbance inputs
r=zeros(ny, time_steps);
r(1,:)=ones(1, time_steps);

d=zeros(nu, time_steps);
if input_dist==1
    d(1,:)=ones(1, time_steps);
end
n=zeros(ny, time_steps);
rand_phase=rand(ny,1);
if meas_noise==1
    n(1,:)=0.1*sin(1e4*time+rand);
    n(2,:)=0.1*sin(1e4*time+rand);
    n(3,:)=0.1*sin(1e4*time+rand);
end
%% Time domain plots with reference and noise and input together
[So,Si,Gp]=get_loop_tfs(G,K);
figure()
lsim(Gp(1:ny,:),[r+n;d],time,'r')
title('Error')

figure()
lsim(Gp(ny+1:ny+nu,:),[r+n;d],time,'r')
title('Input')

%% Plots for reference tracking vs Disturbance rejection tradeoff
% Plot step references separately i.e either a step at ref or at
% disturbance

figure()
subplot(221);
lsim(eye(ny)-So,r,time,'r')
title('y for step at r')

subplot(222);
lsim(G*Si,d,time,'r')
title('y for step at d')

subplot(223);
lsim(K*So,r,time,'r')
title('u for step at r')

subplot(224);
lsim(K*G*Si,d,time,'r')
title('u for step at d')
end