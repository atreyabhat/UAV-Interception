function [A B] = getAB()
%% Define symbolic variables
syms x1 x2 x3 vx vy vz phi theta psi wx wy wz er1 er2 er3 x1d x2d x3d
z = [x1; x2; x3; phi; theta; psi; vx; vy; vz; wx; wy; wz; er1; er2; er3];

syms u1 u2 u3 u4
u = [u1; u2; u3; u4];

syms g m L sigma

syms r1 r2 r3 n1 n2 n3
disturbances = [r1; r2; r3; 0; 0; 0; 0; 0; 0; 0; 0; 0; n1; n2; n3; 0; 0; 0];

syms Ixx Iyy Izz
inertia_matrix = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];

%% fixed body coordinate frame basis
c1 = [1;0;0];
c2 = [0;1;0];
c3 = [0;0;1];

%% Reference Frame
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

%% Define rotation matrices
R = [(cos(theta) * cos(psi)), (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)), (sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi));
       (cos(theta) * sin(psi)), (cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)), (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
      (-sin(theta)), (sin(phi) * cos(theta)), (cos(phi) * cos(theta))];

T_inverse = [1, sin(phi) * tan(theta), cos(phi) * tan(theta);
             0, cos(phi), -sin(phi);
             0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

%% Define system dynamics
z_dot = [vx; vy; vz;
             T_inverse * [wx; wy; wz];
             -g*e3 + (1/m) * R*(u(1) + u(2) + u(3) + u(4))*c3 + (1/m) * R * disturbances(1:3);
             inertia_matrix \ R*((u(2)-u(4))*L*c1 + (u(3)-u(1))*L*c2 + (u(1) -u(2) + u(3) -u(4))*sigma *c3 + [n1; n2; n3] ...
             - cross([wx; wy; wz],inertia_matrix*[wx; wy; wz]));
             (x1d-x1);(x2d-x2);(x3d-x3)];

%% Calculate Jacobians
A = jacobian(z_dot, z);
A = subs(A,{vx,vy, vz, phi, theta, psi, wx, wy, wz, u1, u2, u3, u4, r1, r2, r3, n1, n2, n3}, {0,0,0,0,0,0,0,0,0,m*g/4,m*g/4,m*g/4,m*g/4,0,0,0,0,0,0});
B = jacobian(z_dot, u);
B = subs(B,{vx,vy, vz, phi, theta, psi, wx, wy, wz, u1, u2, u3, u4, r1, r2, r3, n1, n2, n3}, {0,0,0,0,0,0,0,0,0,m*g/4,m*g/4,m*g/4,m*g/4,0,0,0,0,0,0});

% disp (A);
% disp (B);
% 
% %% Check Controllability
% n = size(A, 1);
% m = size(B, 2);
% C = B;
% for i = 1:n-1
%     C = [C, A^i * B];
% end
% 
% rank_C = rank(C);
% disp("Rank of controllability matrix C is:");
% disp(rank_C);
% 
% if rank_C == n
%     disp("The system is controllable.");
% else
%     disp("The system is not fully controllable.");
% end
