function [qd] = stateToQd(x)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 12 vector of state variables [pos rot vel omega]
% qd is a struct including the fields pos, vel, euler, and omega

%current state
qd.pos = x(1:3);
qd.vel = x(7:9);

%Rot = QuatToRot(x(7:10)');
phi = x(4);
theta = x(5);
psi = x(6);

qd.rot = [phi; theta; psi];
qd.omega = x(10:12);

end
