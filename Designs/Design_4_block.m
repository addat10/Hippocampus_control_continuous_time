function [filters,K,hinf]=Design_4_block(G)
nx=size(G.A,1);
nu=size(G.B,2);
ny=size(G.C,1);

%%  Define filters
ws=makeweight(2e3,1,1e-2);
%ws=ss(1);
Ws=ws*diag([1,1,1,0.0001,0.001,0.0015,0.001]);

%wk=1*makeweight(0.5,1e2,1e2);
wk=2.5*ss(1);
wk=2.5*makeweight(0.999,100,1e4);
Wk=wk*eye(nu);
Vr = ss(1)*eye(ny);
Vd = 1e-4*ss(1)*eye(nu);


%% Convert to state space
Ws=ss(Ws);
Wk=ss(Wk);
Vd=ss(Vd);
Vr=ss(Vr);

filters=struct;
filters.Ws=Ws;
filters.Wk=Wk;
filters.Vd=Vd;
filters.Vr=Vr;

%% Plot combined shaping filters

% figure()
% subplot(221);   sigma(inv(Ws*Vr),'r-'); title('inv(Ws*Vr)')
% subplot(222);   sigma(inv(Ws*Vd),'r-'); title('inv(Ws*Vd)')
% subplot(223);   sigma(inv(Wk*Vr),'r-'); title('inv(Wk*Vr)')
% subplot(224);   sigma(inv(Wk*Vd),'r-'); title('inv(Wk*Vd)')
%% Construct generalized plant with  Gd and prefilters

% r --->| Vr |--------------------------+
%                                       |
% d --->| Vd |---------+                |
%                      |                |
%           +--------- | -------------- | ------| Wk |-------> zk
%           |          |                |
%           |          v     _____      v   e
% u --------+--------->O--->|  G  |---->O---+---| Ws |-------> zs
%                            -----  y  -    |
%                                           +----------------> e

systemnames = 'G Ws Wk Vr Vd';
inputvar = ['[r(',num2str(ny),'); d(',num2str(nu),'); u(',num2str(nu),')]'];
input_to_G  = '[u+Vd]';
input_to_Wk = '[u]';
input_to_Ws = '[Vr-G]';
input_to_Vr = '[r]';
input_to_Vd = '[d]';
outputvar   = '[Wk; Ws; Vr-G]';
% cleanupsysic = 'yes';

GSYS = sysic;
%% Design the controller

% Controller design
[K,CL,hinf] = hinfsyn( GSYS, ny, nu, 'Method', 'LMI' );
%[K,CL,hinf] = hinfsyn( GSYS, ny, nu);
hinf

% [K,CL,hinf] = hinfsyn( GSYS, ny, nu, 1.05 * hinf ,'Method', 'LMI' );
% hinf


% with LMIS
% tol=1e-16; regularize=1; relax=1.05;
% nmeas=ny; ncont=nu;
% [K_cvx,hinf_cvx] = cvx_Hinf_OFB(GSYS,nmeas,ncont,tol,regularize,relax);
% K=K_cvx;


end