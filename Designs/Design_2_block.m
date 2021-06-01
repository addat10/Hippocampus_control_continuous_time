function [filters,K,hinf]=Design_2_block(G)

nx=size(G.A,1);
nu=size(G.B,2);
ny=size(G.C,1);

%%  Define filters
ws=makeweight(1e4,150,1e-2);
Ws=ws*eye(ny);

wk=ss(1.2);
Wk=wk*[1,0,0;0,1,0;0,0,3];

%% Convert to state space
Ws=ss(Ws);
Wk=ss(Wk);
filters=struct;
filters.Ws=Ws;
filters.Wk=Wk;

%% Plot combined shaping filters

figure()
subplot(211);   sigma(inv(Ws),'r-'); title('inv(Ws)')
subplot(212);   sigma(inv(Wk),'r-'); title('inv(Wk)')
%% Construct generalized plant with  Gd and prefilters

% r --->--------------------------------+
%                                       |
% 
%                                       |
%           +-------------------------- | ------| Wk |-------> zk
%           |                           |
%           |              _____        v   e
% u --------+-------------->|  G  |---->O---+---| Ws |-------> zs
%                            -----  y  -    |
%                                           +----------------> e

systemnames = 'G Ws Wk';
inputvar = '[r(3); u(3)]';
input_to_G  = '[u]';
input_to_Wk = '[u]';
input_to_Ws = '[r-G]';
outputvar   = '[Ws; Wk; r-G]';
% cleanupsysic = 'yes';

GSYS = sysic;
%% Design the controller

% Controller design
[K,CL,hinf] = hinfsyn( GSYS, ny, nu, 'Method', 'LMI' );

% these filters below are unused for 2 block design but we return these for
% matching the format of the function with other functions that include a
% four block design
Vr = ss(eye(ny));
Vd = ss(eye(nu));
filters.Vd=Vd;
filters.Vr=Vr;
end