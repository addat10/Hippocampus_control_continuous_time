function [x_next]=propagate_state(G,T,x,u)
    % Update controller states
    x_next=x+T*G.A*x+T*G.B*u;    
end