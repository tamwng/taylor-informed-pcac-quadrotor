function metrics = performance_metrics(result, p)
%PERFORMANCE_METRICS Closed-loop, identification, timing, and constraint metrics.

tracking_error = result.x - result.reference;
position_error = tracking_error(1:3,:);
attitude_error = tracking_error(4:6,:);

metrics.position_rmse_axis = sqrt(mean(position_error.^2,2));
metrics.position_rmse = sqrt(mean(sum(position_error.^2,1)));
metrics.attitude_rmse_axis_rad = sqrt(mean(attitude_error.^2,2));
metrics.attitude_rmse_rad = sqrt(mean(sum(attitude_error.^2,1)));
metrics.attitude_rmse_deg = metrics.attitude_rmse_rad*180/pi;
metrics.maximum_tracking_error = max(sqrt(sum(position_error.^2,1)));

prediction_groups = {1:3,4:6,7:9,10:12};
prediction_names = {'position','attitude','velocity','angular_rate'};
for j = 1:numel(prediction_groups)
    values = result.prediction_error(prediction_groups{j},:);
    valid = all(isfinite(values),1);
    if any(valid)
        group_rmse = sqrt(mean(sum(values(:,valid).^2,1)));
    else
        group_rmse = nan;
    end
    metrics.prediction_rmse.(prediction_names{j}) = group_rmse;
end

mass_error = result.mass_hat-result.mass_true;
trim_error = result.trim_thrust_used-result.mass_true*p.g;
metrics.mass_estimation_rmse = sqrt(mean(mass_error.^2));
metrics.mass_estimation_final_abs_error = abs(mass_error(end));
metrics.trim_thrust_rmse = sqrt(mean(trim_error.^2));
metrics.trim_thrust_final_abs_error = abs(trim_error(end));

if strcmpi(result.case_name,'mass_change')
    post = result.time >= p.case1.mass_change_time;
    metrics.mass_estimation_post_change_rmse = sqrt(mean(mass_error(post).^2));
    metrics.trim_thrust_post_change_rmse = sqrt(mean(trim_error(post).^2));
else
    metrics.mass_estimation_post_change_rmse = metrics.mass_estimation_rmse;
    metrics.trim_thrust_post_change_rmse = metrics.trim_thrust_rmse;
end

applied_input = result.u(:,1:end-1);
input_difference = diff(result.u,1,2);
metrics.accumulated_input_variation = sum(abs(input_difference(:)));

valid_time = isfinite(result.qp_time);
metrics.mean_qp_time = mean(result.qp_time(valid_time));
metrics.maximum_qp_time = max(result.qp_time(valid_time));
metrics.solver_failures = sum(result.solver_failure);

state_residual = p.Hx*(result.x-p.xe)-p.hx;
state_violation = max(state_residual,0);
metrics.state_violation_count = sum(any(state_violation > p.qp.safety_tolerance,1));
metrics.maximum_state_violation = max(state_violation(:));

input_upper = applied_input-p.u_max;
input_lower = p.u_min-applied_input;
input_violation = max([input_upper;input_lower],0);
metrics.input_violation_count = sum(any(input_violation > p.qp.safety_tolerance,1));
metrics.maximum_input_violation = max(input_violation(:));

rate_upper = input_difference-p.du_max;
rate_lower = p.du_min-input_difference;
rate_violation = max([rate_upper;rate_lower],0);
metrics.rate_violation_count = sum(any(rate_violation > p.qp.safety_tolerance,1));
metrics.maximum_rate_violation = max(rate_violation(:));

finite_residual = result.qp_constraint_residual(isfinite(result.qp_constraint_residual));
finite_slack = result.qp_max_slack(isfinite(result.qp_max_slack));
if isempty(finite_residual)
    metrics.maximum_qp_constraint_residual = nan;
else
    metrics.maximum_qp_constraint_residual = max(finite_residual);
end
if isempty(finite_slack)
    metrics.maximum_qp_slack = nan;
else
    metrics.maximum_qp_slack = max(finite_slack);
end
end
