clear; close all; clc;

root_directory = fileparts(mfilename('fullpath'));
addpath(root_directory);
if exist('quadprog','file') ~= 2
    error('main_pcac_quadrotor:Dependency', ...
        'quadprog from Optimization Toolbox is required.');
end
p = pcac_parameters();
p.output.directory = fullfile(root_directory,p.output.directory);
if ~exist(p.output.directory,'dir')
    mkdir(p.output.directory);
end
rng(p.seed,'twister');

case1_specs = struct('D',{1,2,3},'fixed_trim',{false,false,false});
if p.case1.include_fixed_trim_baseline
    case1_specs(end+1) = struct('D',p.case1.fixed_trim_baseline_degree,'fixed_trim',true);
end
case2_specs = struct('D',{1,2,3},'fixed_trim',{false,false,false});

case1_noise = measurement_noise_sequence(p.case1,p,p.seed+101);
case2_noise = measurement_noise_sequence(p.case2,p,p.seed+202);
case1_disturbance = disturbance_sequence(p.case1,p);
case2_disturbance = disturbance_sequence(p.case2,p);

case1_results = cell(1,numel(case1_specs));
for j = 1:numel(case1_specs)
    fprintf('Running Case 1, D=%d, fixed trim=%d ...\n', ...
        case1_specs(j).D,case1_specs(j).fixed_trim);
    case1_results{j} = simulate_run(p,p.case1,case1_specs(j), ...
        case1_noise,case1_disturbance);
end

case2_results = cell(1,numel(case2_specs));
for j = 1:numel(case2_specs)
    fprintf('Running Case 2, D=%d ...\n',case2_specs(j).D);
    case2_results{j} = simulate_run(p,p.case2,case2_specs(j), ...
        case2_noise,case2_disturbance);
end

plot_results(case1_results,p,p.case1.name);
plot_results(case2_results,p,p.case2.name);

if p.graphics.make_animation

    animate_quadrotor_comparison( ...
        case1_results(1:3), ...
        p, ...
        'case1_D1_D2_D3_comparison');

    animate_quadrotor_comparison( ...
        case2_results(1:3), ...
        p, ...
        'case2_D1_D2_D3_comparison');

end

all_results = [case1_results,case2_results];
metrics_table = build_metrics_table(all_results);
disp(metrics_table);
writetable(metrics_table,fullfile(p.output.directory,p.output.metrics_file));

results.case1 = case1_results;
results.case2 = case2_results;
save(fullfile(p.output.directory,p.output.mat_file),'results','metrics_table','p','-v7.3');
fprintf('Results written to %s\n',p.output.directory);

function result = simulate_run(p,case_parameters,spec,noise,disturbance_history)
D = spec.D;
dict = taylor_dictionary(D);
rls = rowwise_rls('initialize',[],dict,[],[],[],p);
[vrf,~,~,~] = common_vrf_update([],[],p);

time = 0:p.Ts:case_parameters.Tf;
number_of_samples = numel(time);
number_of_intervals = number_of_samples-1;

x = zeros(12,number_of_samples);
y = zeros(12,number_of_samples);
u = zeros(4,number_of_samples);
prediction_error = nan(12,number_of_samples);
mass_hat = nan(1,number_of_samples);
gravity_residual_hat = nan(1,number_of_samples);
trim_thrust_hat = nan(1,number_of_samples);
trim_thrust_used = nan(1,number_of_samples);
lambda = ones(1,number_of_samples);
beta = ones(1,number_of_samples);
qp_time = nan(1,number_of_samples);
qp_exitflag = nan(1,number_of_samples);
qp_constraint_residual = nan(1,number_of_samples);
qp_max_slack = nan(1,number_of_samples);
solver_failure = false(1,number_of_samples);

x(:,1) = case_parameters.x0;
y(:,1) = x(:,1)+noise(:,1);
u(:,1) = case_parameters.u0;
reference = generate_reference(case_parameters.name,time,p);
mass_true = true_mass_profile(case_parameters,time,p);
gravity_residual_true = p.g*(p.m0./mass_true-1);

for k = 1:number_of_samples
    measured_shifted_state = y(:,k)-p.xe;

    if k > 1
        previous_z = [y(:,k-1)-p.xe; u(:,k-1)-p.ue0];
        predicted_state = identified_map_jacobian(rls,dict,previous_z);
        prediction_error(:,k) = measured_shifted_state-predicted_state;
        [vrf,beta(k),lambda(k)] = common_vrf_update( ...
            vrf,prediction_error(:,k),p);
        rls = rowwise_rls('update',rls,dict,previous_z, ...
            prediction_error(:,k),beta(k),p);
    end

    estimate = estimate_trim_mass(rls,dict,p);
    mass_hat(k) = estimate.mass;
    gravity_residual_hat(k) = estimate.beta_g;
    trim_thrust_hat(k) = estimate.trim_thrust;
    if spec.fixed_trim
        trim_input = p.ue0;
    else
        trim_input = estimate.input;
    end
    trim_thrust_used(k) = trim_input(1);

    if k <= number_of_intervals
        current_z = [measured_shifted_state; u(:,k)-p.ue0];
        [~,A,B,c] = identified_map_jacobian(rls,dict,current_z);
        shifted_trim = trim_input-p.ue0;
        d = c+B*shifted_trim;

        prediction_time = time(k)+(1:p.N)*p.Ts;
        reference_horizon = generate_reference(case_parameters.name,prediction_time,p)-p.xe;
        [u(:,k+1),qp_info] = solve_pcac_qp(A,B,d,measured_shifted_state, ...
            u(:,k),trim_input,reference_horizon,p);

        qp_time(k) = qp_info.solve_time;
        qp_exitflag(k) = qp_info.exitflag;
        qp_constraint_residual(k) = qp_info.constraint_residual;
        qp_max_slack(k) = qp_info.max_slack;
        solver_failure(k) = ~qp_info.success;

        current_disturbance.force_inertial = disturbance_history(1:3,k);
        current_disturbance.torque_body = disturbance_history(4:6,k);
        x(:,k+1) = rk4_step(x(:,k),u(:,k),mass_true(k),p,current_disturbance);
        y(:,k+1) = x(:,k+1)+noise(:,k+1);
    end
end

result.case_name = case_parameters.name;
result.case_title = case_parameters.title;
result.D = D;
result.fixed_trim = spec.fixed_trim;
if spec.fixed_trim
    result.label = sprintf('D=%d fixed trim',D);
else
    result.label = sprintf('D=%d',D);
end
result.dictionary_coefficients = dict.total_coefficients;
result.time = time;
result.x = x;
result.y = y;
result.u = u;
result.reference = reference;
result.prediction_error = prediction_error;
result.mass_true = mass_true;
result.mass_hat = mass_hat;
result.gravity_residual_true = gravity_residual_true;
result.gravity_residual_hat = gravity_residual_hat;
result.trim_thrust_hat = trim_thrust_hat;
result.trim_thrust_used = trim_thrust_used;
result.lambda = lambda;
result.beta = beta;
result.qp_time = qp_time;
result.qp_exitflag = qp_exitflag;
result.qp_constraint_residual = qp_constraint_residual;
result.qp_max_slack = qp_max_slack;
result.solver_failure = solver_failure;
result.metrics = performance_metrics(result,p);
end

function mass = true_mass_profile(case_parameters,time,p)
if strcmpi(case_parameters.name,'mass_change')
    mass = p.m0*ones(size(time));
    mass(time >= case_parameters.mass_change_time) = case_parameters.mass_after;
elseif strcmpi(case_parameters.name,'helix')
    mass = case_parameters.mass*ones(size(time));
else
    error('main_pcac_quadrotor:Case','Unknown case.');
end
end

function noise = measurement_noise_sequence(case_parameters,p,seed)
time = 0:p.Ts:case_parameters.Tf;
noise = zeros(12,numel(time));
if ~p.measurement_noise.enabled
    return
end
standard_deviation = p.measurement_noise.std(:);
if all(standard_deviation == 0)
    standard_deviation = [p.measurement_noise.std_position*ones(3,1); ...
                          p.measurement_noise.std_angle*ones(3,1); ...
                          p.measurement_noise.std_velocity*ones(3,1); ...
                          p.measurement_noise.std_rate*ones(3,1)];
end
stream = RandStream('mt19937ar','Seed',seed);
noise = standard_deviation.*randn(stream,12,numel(time));
end

function disturbance = disturbance_sequence(case_parameters,p)
time = 0:p.Ts:(case_parameters.Tf-p.Ts);
force = case_parameters.disturbance.force_amplitude(:).*sin( ...
    case_parameters.disturbance.force_frequency(:)*time + ...
    case_parameters.disturbance.force_phase(:));
torque = case_parameters.disturbance.torque_amplitude(:).*sin( ...
    case_parameters.disturbance.torque_frequency(:)*time + ...
    case_parameters.disturbance.torque_phase(:));
disturbance = [force;torque];
end

function metrics_table = build_metrics_table(results)
number_of_runs = numel(results);
Case = cell(number_of_runs,1);
Method = cell(number_of_runs,1);
Coefficients = zeros(number_of_runs,1);
PositionRMSE_m = zeros(number_of_runs,1);
AttitudeRMSE_deg = zeros(number_of_runs,1);
PredictionPositionRMSE = zeros(number_of_runs,1);
PredictionAttitudeRMSE = zeros(number_of_runs,1);
PredictionVelocityRMSE = zeros(number_of_runs,1);
PredictionRateRMSE = zeros(number_of_runs,1);
MassPostChangeRMSE_kg = zeros(number_of_runs,1);
TrimPostChangeRMSE_N = zeros(number_of_runs,1);
FinalMassError_kg = zeros(number_of_runs,1);
FinalTrimError_N = zeros(number_of_runs,1);
MaximumTrackingError_m = zeros(number_of_runs,1);
InputVariation = zeros(number_of_runs,1);
MeanQPTime_ms = zeros(number_of_runs,1);
MaximumQPTime_ms = zeros(number_of_runs,1);
StateViolationCount = zeros(number_of_runs,1);
MaximumStateViolation = zeros(number_of_runs,1);
InputViolationCount = zeros(number_of_runs,1);
MaximumInputViolation = zeros(number_of_runs,1);
RateViolationCount = zeros(number_of_runs,1);
MaximumRateViolation = zeros(number_of_runs,1);
SolverFailures = zeros(number_of_runs,1);

for j = 1:number_of_runs
    result = results{j};
    m = result.metrics;
    Case{j} = result.case_name;
    Method{j} = result.label;
    Coefficients(j) = result.dictionary_coefficients;
    PositionRMSE_m(j) = m.position_rmse;
    AttitudeRMSE_deg(j) = m.attitude_rmse_deg;
    PredictionPositionRMSE(j) = m.prediction_rmse.position;
    PredictionAttitudeRMSE(j) = m.prediction_rmse.attitude;
    PredictionVelocityRMSE(j) = m.prediction_rmse.velocity;
    PredictionRateRMSE(j) = m.prediction_rmse.angular_rate;
    MassPostChangeRMSE_kg(j) = m.mass_estimation_post_change_rmse;
    TrimPostChangeRMSE_N(j) = m.trim_thrust_post_change_rmse;
    FinalMassError_kg(j) = m.mass_estimation_final_abs_error;
    FinalTrimError_N(j) = m.trim_thrust_final_abs_error;
    MaximumTrackingError_m(j) = m.maximum_tracking_error;
    InputVariation(j) = m.accumulated_input_variation;
    MeanQPTime_ms(j) = 1e3*m.mean_qp_time;
    MaximumQPTime_ms(j) = 1e3*m.maximum_qp_time;
    StateViolationCount(j) = m.state_violation_count;
    MaximumStateViolation(j) = m.maximum_state_violation;
    InputViolationCount(j) = m.input_violation_count;
    MaximumInputViolation(j) = m.maximum_input_violation;
    RateViolationCount(j) = m.rate_violation_count;
    MaximumRateViolation(j) = m.maximum_rate_violation;
    SolverFailures(j) = m.solver_failures;
end

metrics_table = table(Case,Method,Coefficients,PositionRMSE_m,AttitudeRMSE_deg, ...
    PredictionPositionRMSE,PredictionAttitudeRMSE,PredictionVelocityRMSE, ...
    PredictionRateRMSE,MassPostChangeRMSE_kg,TrimPostChangeRMSE_N, ...
    FinalMassError_kg,FinalTrimError_N,MaximumTrackingError_m,InputVariation,MeanQPTime_ms,MaximumQPTime_ms, ...
    StateViolationCount,MaximumStateViolation,InputViolationCount, ...
    MaximumInputViolation,RateViolationCount,MaximumRateViolation,SolverFailures);
end
