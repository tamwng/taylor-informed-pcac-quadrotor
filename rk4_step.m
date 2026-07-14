function xnext = rk4_step(x, u, mass, p, disturbance)
%RK4_STEP Fixed-step RK4 with zero-order-held input over one sample.

h = p.Ts/p.rk4_substeps;
xnext = x(:);
for j = 1:p.rk4_substeps
    k1 = quadrotor_dynamics(xnext,             u, mass, p, disturbance);
    k2 = quadrotor_dynamics(xnext + 0.5*h*k1, u, mass, p, disturbance);
    k3 = quadrotor_dynamics(xnext + 0.5*h*k2, u, mass, p, disturbance);
    k4 = quadrotor_dynamics(xnext + h*k3,     u, mass, p, disturbance);
    xnext = xnext + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end
end
