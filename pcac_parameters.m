function p = pcac_parameters()
%PCAC_PARAMETERS Reproducible configuration for both simulation cases.

p.seed = 27;
p.g = 9.81;
p.m0 = 4.34;
p.J = diag([0.0820, 0.0845, 0.1377]);
p.Ts = 0.10;
p.rk4_substeps = 10;
p.euler_min_abs_cos_roll = 0.08;

p.xe = zeros(12,1);
p.ue0 = [p.m0*p.g; zeros(3,1)];

p.u_min = [0; -2.0; -2.0; -1.2];
p.u_max = [85; 2.0; 2.0; 1.2];
p.du_max = [6.0; 0.35; 0.35; 0.25];
p.du_min = -p.du_max;

angle_limit = [pi; 50*pi/180; 50*pi/180];
Cxi = zeros(3,12);
Cxi(:,4:6) = eye(3);
p.Hx = [Cxi; -Cxi];
p.hx = [angle_limit; angle_limit];

p.N = 10;
p.Q = diag([110, 110, 150, 2.0, 3.0, 3.0, ...
             18, 18, 28, 0.7, 0.7, 0.7]);
p.Qf = 8*p.Q;
p.R = diag([0.015, 0.18, 0.18, 0.12]);
p.S = 1.0e5*eye(size(p.Hx,1));

p.qp.algorithm = 'active-set';
p.qp.max_iterations = 180;
p.qp.constraint_tolerance = 1.0e-7;
p.qp.hessian_regularization = 1.0e-9;
p.qp.safety_tolerance = 2.0e-7;

p.rls.type = 'VRF';
p.rls.P0 = 0.1;
p.rls.lambda = 0.995;
p.rls.tau_n = 5;
p.rls.tau_d = 25;
p.rls.eta = 0.50;
p.rls.beta_max = inf;
p.rls.jitter = 1.0e-12;
p.rls.covariance_eigenvalue_max = inf;
p.rls.initialization = 'nominal-linear-zoh';

p.mass_bounds = [2.5, 7.0];

p.measurement_noise.enabled = false;
p.measurement_noise.std = zeros(12,1);
p.measurement_noise.std_position = 0.002;
p.measurement_noise.std_angle = 0.05*pi/180;
p.measurement_noise.std_velocity = 0.005;
p.measurement_noise.std_rate = 0.08*pi/180;

p.case1.name = 'mass_change';
p.case1.title = 'Case 1: abrupt mass change';
p.case1.Tf = 28;
p.case1.x0 = zeros(12,1);
p.case1.u0 = p.ue0;
p.case1.mass_change_time = 13;
p.case1.mass_after = 5.60;
p.case1.include_fixed_trim_baseline = true;
p.case1.fixed_trim_baseline_degree = 3;
p.case1.disturbance.force_amplitude = zeros(3,1);
p.case1.disturbance.force_frequency = [0.31; 0.27; 0.23];
p.case1.disturbance.force_phase = [0.2; 1.1; -0.4];
p.case1.disturbance.torque_amplitude = zeros(3,1);
p.case1.disturbance.torque_frequency = [0.37; 0.33; 0.29];
p.case1.disturbance.torque_phase = [-0.5; 0.6; 0.9];
p.case1.reference.ramp_time = 3.0;
p.case1.reference.amplitude_xy = [0.80; 0.60];
p.case1.reference.frequency_xy = [0.36; 0.29];
p.case1.reference.z_offset = 1.00;
p.case1.reference.z_amplitude = 0.70;
p.case1.reference.z_frequency = 0.90;

p.case2.name = 'helix';
p.case2.title = 'Case 2: aggressive helix tracking';
p.case2.Tf = 21;
p.case2.x0 = zeros(12,1);
p.case2.u0 = p.ue0;
p.case2.mass = p.m0;
p.case2.include_fixed_trim_baseline = false;
p.case2.disturbance.force_amplitude = zeros(3,1);
p.case2.disturbance.force_frequency = [0.41; 0.35; 0.28];
p.case2.disturbance.force_phase = [0.7; -0.2; 1.3];
p.case2.disturbance.torque_amplitude = zeros(3,1);
p.case2.disturbance.torque_frequency = [0.39; 0.32; 0.24];
p.case2.disturbance.torque_phase = [0.1; 0.9; -0.7];
p.case2.reference.ramp_time = 3.0;
p.case2.reference.radius = 1.50;
p.case2.reference.angular_rate = 1.50;
p.case2.reference.z_offset = 0.80;
p.case2.reference.climb_rate = 0.075;
p.case2.reference.z_amplitude = 0.22;
p.case2.reference.z_frequency = 0.78;

p.graphics.visible = 'on';
p.graphics.save_figures = true;
p.graphics.save_matlab_figures = false;
p.graphics.make_animation = true;
p.graphics.animation_degree = 3;
p.graphics.animation_stride = 2;
p.graphics.animation_fps = 18;
p.graphics.arm_length = 0.32;

p.output.directory = 'results';
p.output.mat_file = 'pcac_quadrotor_results.mat';
p.output.metrics_file = 'pcac_quadrotor_metrics.csv';
end
