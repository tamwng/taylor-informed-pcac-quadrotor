function estimate = estimate_trim_mass(rls, dict, p)
%ESTIMATE_TRIM_MASS Vertical-row projection and trim estimates, paper Eqs. (52)-(56).

q = 9;
constant_index = find(strcmp(dict.names{q},'1'),1);
thrust_index = find(strcmp(dict.names{q},'ftilde'),1);
if isempty(constant_index) || isempty(thrust_index)
    error('estimate_trim_mass:Dictionary','The v3 row must contain 1 and ftilde.');
end

constant_coefficient = rls.theta{q}(constant_index);
thrust_coefficient_raw = rls.theta{q}(thrust_index);

mass_min = min(p.mass_bounds);
mass_max = max(p.mass_bounds);
thrust_coefficient_min = p.Ts/mass_max;
thrust_coefficient_max = p.Ts/mass_min;
thrust_coefficient = min(max(thrust_coefficient_raw, ...
    thrust_coefficient_min),thrust_coefficient_max);

alpha_f = thrust_coefficient/p.Ts;
beta_g = constant_coefficient/p.Ts;
mass_hat = 1/alpha_f;
shifted_trim = -beta_g/alpha_f;
trim_thrust_unbounded = p.m0*p.g + shifted_trim;
trim_thrust = min(max(trim_thrust_unbounded,p.u_min(1)),p.u_max(1));

estimate.constant_coefficient = constant_coefficient;
estimate.thrust_coefficient_raw = thrust_coefficient_raw;
estimate.thrust_coefficient = thrust_coefficient;
estimate.alpha_f = alpha_f;
estimate.beta_g = beta_g;
estimate.mass = mass_hat;
estimate.shifted_trim = shifted_trim;
estimate.trim_thrust_unbounded = trim_thrust_unbounded;
estimate.trim_thrust = trim_thrust;
estimate.input = [trim_thrust; zeros(3,1)];
end
