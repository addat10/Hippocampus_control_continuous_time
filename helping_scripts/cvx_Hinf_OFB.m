% This file collects functions to synthesize Hinf output-feedback synthesis
% problem
function [K_ss,hinf_opt] = cvx_Hinf_OFB(GSYS,ncont,nmeas,tol,regularize,relax)
% This function synthesizes a controller that minimizes the closed-loop
% Hinf norm
% Follows Notation from: LMI methods in control by Scherer and Weiland
% Still to be implemented:
%   1. Discrete-time systems
%   2. LPV systems: Continuous and Discrete-time systems
%   3. Eliminate the variables via Dualization Lemma and Elimination Lemma    
    G=get_model_mat(GSYS,ncont,nmeas);
    
        % STEP 1: First solve for minimizing the H2 norm to get an estimate of
        %         achievable H2 norm
        [v,hinf_opt,~,~]=Hinf_optimization_problem_cvx(G,tol,'step1');
    if regularize==1
        % STEP 2: Relax the optimal norm and add this as constraint.
        % Minimize the regularization parameter alpha.
        hinf_opt=relax*hinf_opt;  
        [v,~,alpha_opt,~]=Hinf_optimization_problem_cvx(G,tol,'step2',hinf_opt);

        % STEP 3: Relax the optimal alpha and add this as constraint.
        % Maximize the conditioning parameter beta.
        alpha_opt=relax*alpha_opt;
        [v,~,~,beta_opt]=Hinf_optimization_problem_cvx(G,tol,'step3',hinf_opt,alpha_opt);          
    end
    K_mat=get_control_matrices(v,G);
    n=size(G.A,1);
    K_ss=ss(K_mat(1:n,1:n),K_mat(1:n,n+1:n+nmeas),K_mat(n+1:n+ncont,1:n),K_mat(n+1:n+ncont,n+1:n+nmeas));
end

function [v,hinf_opt,alpha_opt,beta_opt]=Hinf_optimization_problem_cvx(G,tol,method,hinf_opt,alpha_opt,beta_opt)
% This function solves the H2 optimal control problem in the substituted 
% variables v and returns the optimal v
[nx,nw,nz,ncont,nmeas]=get_sizes(G);

cvx_clear
cvx_begin sdp
cvx_precision best

variable X(nx,nx) symmetric
variable Y(nx,nx) symmetric
variable K(nx,nx) 
variable L(nx,nmeas) 
variable M(ncont,nx) 
variable N(ncont,nmeas) 
variable gamma_hinf
variable alpha_reg
variable beta_cond

%% Define new functions of variables for convenience
X_v=[   Y,          eye(nx);...
        eye(nx),    X];
A_v=[   (G.A)*Y+(G.B)*M,        (G.A)+(G.B)*N*(G.C);...
        K,                      X*(G.A)+L*(G.C)];
B_v=[   (G.B1)+(G.B)*N*(G.F);...
        X*(G.B1)+L*(G.F)];
C_v=[   (G.C1)*Y+(G.E)*M,   (G.C1)+(G.E)*N*(G.C)];
D_v=[   (G.D1)+(G.E)*N*(G.F)];
KLMN=[K,L;M,N];

% Define LMIs for usual Hinf synthesis
LMI_1=X_v;
LMI_2=[ A_v+A_v',   B_v,                    C_v';...
        B_v',       -gamma_hinf*eye(nw),    D_v';...
        C_v,        D_v,                    -gamma_hinf*eye(nz)];
    
% Define Regularization LMI
Reg_LMI_1=X-alpha_reg*eye(nx);
Reg_LMI_2=Y-alpha_reg*eye(nx);

Reg_LMI_3=[ alpha_reg*eye(nx+ncont),   KLMN;...
            KLMN',                     alpha_reg*eye(nx+nmeas)];   
Reg_LMI_4=[ Y,                  beta_cond*eye(nx);...
            beta_cond*eye(nx),  X];

switch lower(method)
    case {'step1'}
            minimize gamma_hinf
            subject to:
                LMI_1>=tol*eye(2*nx)
                LMI_2<=-tol*eye(2*nx+nw+nz)
    case {'step2'}
            minimize alpha_reg
            subject to:
                LMI_1>=tol*eye(2*nx)
                LMI_2<=-tol*eye(2*nx+nw+nz)                
                
                Reg_LMI_1<=-tol*eye(nx)
                Reg_LMI_2<=-tol*eye(nx)
                Reg_LMI_3>=tol*eye(2*nx+ncont+nmeas)  
                
                gamma_hinf==hinf_opt
    case {'step3'}
            minimize -1*beta_cond
            subject to:
                LMI_1>=tol*eye(2*nx)
                LMI_2<=-tol*eye(2*nx+nw+nz)
                
                Reg_LMI_1<=-tol*eye(nx)
                Reg_LMI_2<=-tol*eye(nx)
                Reg_LMI_3>=tol*eye(2*(nx+ncont))   
                Reg_LMI_4>=tol*eye(2*nx) 
                
                gamma_hinf==hinf_opt
                alpha_reg==alpha_opt
end        
cvx_end
hinf_opt=gamma_hinf;
alpha_opt=alpha_reg;
beta_opt=beta_cond;
v=struct;
v.X=X;  v.Y=Y;  v.K=K;  v.L=L;  v.M=M;  v.N=N;
end
%%
function [K]=get_control_matrices(v,G)
% This function gives back the controller variables and the Lyapunov matrix 
% from the substitute variables v
    n=size((v.X),1);
    [U,S,V]=svd(eye(n)-(v.X)*(v.Y));
    U=U*sqrt(S);
    V=(sqrt(S)*V')';
    K1=[(v.K)-(v.X)*(G.A)*(v.Y), (v.L);(v.M), (v.N)]; 
    K2=[U,(v.X)*(G.B);zeros(size(v.M,1),n),eye(size((v.M),1))]\K1;
    K3=[V',zeros(n,size((v.L),2));(G.C)*(v.Y),eye(size((v.N)))];
    K=(K3'\K2')';    
end
%%
function [nx,nw,nz,ncont,nmeas]=get_sizes(G)
% Get sizes of signals
    nx=size(G.A,1);
    nw=size(G.B1,2);
    ncont=size(G.B,2);
    nmeas=size(G.C,1);
    nz=size(G.C1,1);
end
function G=get_model_mat(G_genp,ncont,nmeas)
% This function returns model matrices as a struct
[A_gp,B_gp,C_gp,D_gp]=ssdata(G_genp);
nx=size(A_gp,1);
nw=size(B_gp,2)-ncont;
nz=size(C_gp,1)-nmeas;

G=struct;
G.A=A_gp;
G.B1=B_gp(:,1:nw);
G.B=B_gp(:,nw+1:end);
G.C1=C_gp(1:nz,:);
G.C=C_gp(nz+1:end,:);
G.D1=D_gp(1:nz,1:nw);
G.E=D_gp(1:nz,nw+1:end);
G.F=D_gp(nz+1:end,1:nw);
end

