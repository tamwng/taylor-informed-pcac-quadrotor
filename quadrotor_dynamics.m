function xdot = quadrotor_dynamics(x, u, mass, p, disturbance)
%QUADROTOR_DYNAMICS Exact intrinsic Z-X-Y quadrotor dynamics, paper Eq. (16).

if nargin < 5 || isempty(disturbance)
    disturbance.force_inertial = zeros(3,1);
    disturbance.torque_body = zeros(3,1);
end

x = x(:);
u = u(:);
position = x(1:3); %#ok<NASGU>
psi = x(4);
phi = x(5);
theta = x(6);
velocity = x(7:9);
omega = x(10:12);

cpsi = cos(psi);   spsi = sin(psi);
cphi = cos(phi);   sphi = sin(phi);
ctheta = cos(theta); stheta = sin(theta);

if abs(cphi) < p.euler_min_abs_cos_roll
    error('quadrotor_dynamics:EulerSingularity', ...
        'The intrinsic Z-X-Y chart is too close to cos(phi)=0 at phi=%.6f rad.', phi);
end

R = [cpsi*ctheta - sphi*spsi*stheta, -spsi*cphi, cpsi*stheta + sphi*spsi*ctheta; ...
     spsi*ctheta + sphi*cpsi*stheta,  cpsi*cphi, spsi*stheta - sphi*cpsi*ctheta; ...
     -cphi*stheta,                    sphi,      cphi*ctheta];

Einv = [-stheta/cphi, 0, ctheta/cphi; ...
         ctheta,      0, stheta; ...
         stheta*tan(phi), 1, -ctheta*tan(phi)];

e3 = [0;0;1];
f = u(1);
tau = u(2:4);

position_dot = velocity;
euler_dot = Einv*omega;
velocity_dot = -p.g*e3 + (f/mass)*(R*e3) + disturbance.force_inertial(:)/mass;
omega_dot = p.J\(-cross(omega, p.J*omega) + tau + disturbance.torque_body(:));

xdot = [position_dot; euler_dot; velocity_dot; omega_dot];
end
