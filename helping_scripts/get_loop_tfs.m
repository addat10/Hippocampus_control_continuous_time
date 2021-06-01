function [So,Si,Gp]=get_loop_tfs(G,K)
ny=size(G.C,1);
nu=size(K.C,1);
So=feedback(eye(ny),G*K);
Si=feedback(eye(nu),K*G);
Gp=[So,     G*Si;...
    K*So,   K*G*Si];
end