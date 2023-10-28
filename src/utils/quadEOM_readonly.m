function sdot = quadEOM_readonly(t, s, u, params)
% QUADEOM_READONLY Solve quadrotor equation of motion
%   quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% t      - 1 x 1, time
% s      - 12 x 1, state vector = [x, y, z, phi, theta, psi, vx, vy, vz, w1, w2, w3]
% F      - 1 x 4, thrust output from controller (only used in simulation)
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from nanoplus() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 12 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, nanoplus





% Assign states
x = s(1);
y = s(2);
z = s(3);
phi = s(4);
theta = s(5);
psi = s(6);
vx = s(7);
vy = s(8);
vz = s(9);
w1 = s(10);
w2 = s(11);
w3 = s(12);




%states 1-3
xdot = vx;
ydot = vy;
zdot = vz;


w = [w1; w2; w3];
alpha = [phi theta psi];

T_inverse = [ 1, sin(alpha(1))*tan(alpha (2)),cos(alpha (1))*tan(alpha(2));
0, cos(alpha (1)),-sin(alpha(1));
0, sin(alpha(1))/ cos(alpha(2)), cos(alpha(1))/cos(alpha(2))];

Rce = [(cos(alpha(2)) * cos(alpha(3))), (sin(alpha(1))* sin(alpha(2)) *cos(alpha(3)) - cos(alpha(1)) * sin(alpha(3))), (sin(alpha(1))*sin(alpha(3)) + cos(alpha(1)) *sin(alpha(2))*cos(alpha(3)));
(cos (alpha(2)) * sin(alpha(3))), (cos(alpha(1)) * cos(alpha(3)) + sin(alpha(1))*sin (alpha(2))*sin (alpha(3))), (( cos(alpha(1))*sin(alpha (2))*sin(alpha(3))) - (sin(alpha(1))*cos(alpha(3))));
(-sin (alpha (2))), (sin (alpha(1)) * cos (alpha(2))), (cos(alpha(1)) * cos(alpha(2)))];

%states 4-6
alphadot = T_inverse*w;

%r = [r1; r2; r3];
r = zeros(1,3);
%u = [u1; u2; u3; u4];
u = zeros(1,4);
%n = [n1; n2; n3];
n = zeros(1,3);

%states 7-9
v_dot = -params.gravity + (1/params.mass) .* Rce .* (u(1) + u(2) + u(3) + u(4)) + (1/params.mass) .* Rce .* r;


%states 10-12
w_dot = params.I \ ((u(2) - u(4)) * params.arm_length + (u(3) - u(1)) * params.arm_length + (u(1) - u(2) + u(3) - u(4)) * params.sigma + n - cross(w, params.I * w));



% Assemble sdot
sdot = zeros(12,1);
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = alphadot(1);
sdot(5)  = alphadot(2);
sdot(6)  = alphadot(3);
sdot(7)  = v_dot(1);
sdot(8)  = v_dot(2);
sdot(9)  = v_dot(3);
sdot(10) = w_dot(1);
sdot(11) = w_dot(2);
sdot(12) = w_dot(3);

end
