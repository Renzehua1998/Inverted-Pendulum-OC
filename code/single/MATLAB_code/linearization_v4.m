clear all
syms theta M m l u x_dot theta_dot x Kv Kt Rw I_rotor N_rotor g r_pulley


q = [x; theta; x_dot; theta_dot];
dq = [x_dot; theta_dot];

% Non-linear System Dynamics
% alpha = Kt/Rw*N_rotor;
% De = [-cos(theta)   l; (M+m)     -m*l*cos(theta)];
% Ce = [0;    m*l*sin(theta)];
% Ge = [-g*sin(theta);    0];
% Be = [alpha;    0];

J_rotor = [I_rotor * r_pulley^2*N_rotor^2    0;
            0                   0];
 
B_damp = [Kv*Kt/Rw*N_rotor^2    0;
            0                   0];

        
% Computing double derivative of q 
ddq = (De + J_rotor) \ (Be * u - Ce - B_damp * dq - Ge);




%% linearization v2
alpha = Kt*N_rotor/(Rw*r_pulley);

J_rotor = [I_rotor * N_rotor^2 / r_pulley^2    0;
            0                   0];
 
B_damp = [Kv*Kt*N_rotor^2/(Rw*r_pulley^2)    0;
            0                   0];

De = [M+m -m*l*cos(theta); -cos(theta) l];
Ce = [m*l*sin(theta);    0];
Ge = [0;    -g*sin(theta)];
Be = [alpha;    0];

ddq = (De + J_rotor) \ (Be * u - Ce - B_damp * dq - Ge);

% Linearization of System 
q_eq = [x; 0;0;0];
u_eq = 0;

lin_x_dot = [dq; ddq];

lin_x_dot_2 = subs(lin_x_dot, [cos(theta), sin(theta)], [1, sym(theta)]);


i = 3;
A32 = diff(lin_x_dot_2(i), theta);
A33 = diff(lin_x_dot_2(i), x_dot);
B31 = diff(lin_x_dot_2(i), u);

i = 4;
A42 = diff(lin_x_dot_2(i), theta); 
A43 = diff(lin_x_dot_2(i), x_dot);
B41 = diff(lin_x_dot_2(i), u);


A_lin = [0 0 1 0;
         0 0 0 1;
         0 A32 A33 0;
         0 A42 A43 0];
    
     
     
B_lin = [0;
         0;
         B31;
         B41];
     
C_lin = [1 0 0 0; 0 1 0 0];
D_lin = 0;

dxdt = q_eq + A_lin * (q - q_eq) + B_lin * (u - u_eq);

% 
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          2*x
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            x
%                                                                                                 (r*m*cos(theta))/(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2) - x*((r*Rw*m*sin(theta) + Rw*l*m*cos(theta) - Rw*g*m*cos(theta)^2 + Rw*g*m*sin(theta)^2 - Kt*Kv*N_rotor^2*m*x_dot*sin(theta))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2)) + ((- I_rotor*m*sin(theta)*N_rotor^2 + 2*m*cos(theta)*sin(theta))*(- Kt*Kv*m*x_dot*cos(theta)*N_rotor^2 + r*Rw*m*cos(theta) - Rw*l*m*sin(theta) + Rw*g*m*cos(theta)*sin(theta)))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2)^2)) - (Kt*Kv*N_rotor^2*m*x*cos(theta))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2))
%  x*((M*Rw*g*cos(theta) + Rw*g*m*cos(theta) - Rw*l*m*cos(theta)^2 + Rw*l*m*sin(theta)^2 + I_rotor*N_rotor^2*Rw*l*m*cos(theta))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2)) - ((- I_rotor*m*sin(theta)*N_rotor^2 + 2*m*cos(theta)*sin(theta))*(r*M*Rw + r*Rw*m + M*Rw*g*sin(theta) + Rw*g*m*sin(theta) - Rw*l*m*cos(theta)*sin(theta) - Kt*Kv*M*N_rotor^2*x_dot - Kt*Kv*N_rotor^2*m*x_dot + I_rotor*N_rotor^2*Rw*l*m*sin(theta)))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2)^2)) + (r*(Rw*m + M*Rw))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2)) - (x*(Kt*Kv*N_rotor^2*m + Kt*Kv*M*N_rotor^2))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2 + M + m - m*cos(theta)^2))
%  
