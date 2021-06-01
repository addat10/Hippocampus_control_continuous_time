function G = hippocampus_approx_model(d)
% This function creates an approximate Hippocampus model with a handle and
% 7 states.
% y1=x1 : position x
% y2=x2 : position y
% y3=x3 : position z
% x4 : forward velocity
% x5 : angular velocity p
% x6 : angular velocity q
% x7 : angular velocity r


% u1=x1 : forward thrust
% u2=x2 : Torque tau_phi
% u3=x3 : Torque tau_theta
% u4=x3 : Torque tau_psi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define Parameters
% zg       =  0.05;    % [m]
% g        =  9.81 ;   % [m/s^2]
mass     =  1.43 ;   % [kg]
Ix       =  0.00241; % [kgm^2]
Iy       =  0.01072; % [kgm^2]
Iz       =  0.01072; % [kgm^2]
Xudot    = -1.11;    % [kg]
Yvdot    = -2.80;    % [kg]
Zwdot    = -2.80 ;   % [kg]
Kpdot    = -0.0018;  % [kgm^2]
Mqdot    = -0.0095;  % [kgm^2]
Nrdot    = -0.0095;  % [kgm^2]
Xu       = -4.56;    % [kg/m]
Yv       = -17.36;   % [kg/m]
Zw       = -17.36;   % [kg/m]
Kp       = -0.0028;  % [kgm^2]
Mq       = -0.0476 ; % [kgm^2]
Nr       = -0.0476;  % [kgm^2]

% Calculate mass matrix
Mrb      = mdiag(mass,mass,mass,Ix,Iy,Iz);
Ma       = -mdiag(Xudot,Yvdot,Zwdot,Kpdot,Mqdot,Nrdot);
M        = Mrb + Ma;
Mi       = inv(M);

% Construct approximate damping matrix (not obtained by linearization due
% to non-differentiability)
Da       = -mdiag(Xu,Yv,Zw,Kp,Mq,Nr);

% Build the blocks of the A Matrix
A11      = zeros(3);
A12      = [1;0;0];
A13      = [0,0,0;...
           0,0,d;...
           0,-d,0];
A21      = zeros(1,3);
A22      = -Mi(1,1)*Da(1,1);
A23      = zeros(1,3);
A31      = zeros(3);
A32      = zeros(3,1);
A33      = -Mi(4:6,4:6)*Da(4:6,4:6);

% Assemble the state space matrices
A        = [A11,A12,A13;...
           A21,A22,A23;...
           A31,A32,A33];
B        = [zeros(3,4);eye(4)];

% State output
C        = eye(7);

% Position output
% C        = [eye(3),zeros(3,4)];

G        = ss(A,B,C,[]);
end
