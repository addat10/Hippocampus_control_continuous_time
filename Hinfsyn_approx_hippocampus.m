% This script is used for for designing an hinf optimal controller for an
% approximate model of the Hippocampus
clear all
close all
clc
addpath('./helping_scripts')
addpath('./Designs')
%% Define G
% Handle length
d = 0.15;     % [m]
G = hippocampus_approx_model(d);

%% Understand G
figure()
sigma(G)
title('G')
%% Design K
[filters,K,hinf]=Design_4_block(G);
%[filters,K,hinf]=Design_2_block_roll_off_K(G);

% Understand K in freq domain
figure()
sigma(K)
title('K')
% Frequency domain analysis
freq_domain_analysis(G,K,filters)
[So,~,~]=get_loop_tfs(G,K);
G_cl=eye(size(So,1))-So;
% Understand G_cl
figure()
sigma(G_cl(1:3,1:3))
title('G closed-loop')
% Time domain analysis
% time_domain_analysis(G,K)

%% Post process controller

% Remove fast poles of the controller
K_red=balred(K,18);
figure()
sigma(K)
hold on
sigma(K_red)
legend('K','K reduced')
title('Reduced controller')

% Understand controller
figure()
pzmap(G)
hold on
pzmap(K,'r')
pzmap(K_red,'g')
legend('G','K','Kred')
title('Pole zero map of plant and controller')



% Save current controller
K2=K;
save('./LTI_controller_continuous_2','K2','d')
%% Test Benchmark trajectory
time_steps=50000;
delta_t=0.01;
test_benchmark_traj(G,K,time_steps,delta_t)
%% LTI system with saturation
time_steps=5000;
delta_t=0.0005;
test_benchmark_traj_with_sat(G,K,time_steps,delta_t)

