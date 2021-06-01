function [u_sat, omega] = saturate_u(u)
  %  u_sat=u;
    h     = 0.0481;   % [m] Distance of rotors to HippoCampus center
    Cd    = 0.0024;   % Coefficient of drag for the rotors
    tlim  = 3;        % Maximal torque of the motors 

    Q = [1,1,1,1;-Cd,Cd,-Cd,Cd;-h,-h,h,h;h,-h,-h,h];
    Qinv = inv(Q);

    % Transform into rotor space
    omega = Qinv*u;

    % Calculate utilization of each rotor
    util  = abs(omega/tlim);

    % Saturate torque at max utilization
    omega = omega ./ max(util, 1);    

    %Transform back into body frame
   u_sat = Q * omega;
end 