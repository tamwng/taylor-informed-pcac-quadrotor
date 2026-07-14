function plot_results(run_collection, p, case_name)
%PLOT_RESULTS Reproduce all comparison plots for one simulation case.

if iscell(run_collection)
    runs = [run_collection{:}];
else
    runs = run_collection;
end
number_of_runs = numel(runs);
colors = lines(number_of_runs);
labels = {runs.label};
time = runs(1).time;
reference = runs(1).reference;

if strcmpi(case_name,'mass_change')
    fig = new_figure('Case 1 estimates',p);
    subplot(2,2,1); hold on; grid on;
    plot(time,runs(1).mass_true,'k--','LineWidth',1.8,'DisplayName','true');
    for j = 1:number_of_runs
        plot(time,runs(j).mass_hat,'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    ylabel('mass (kg)'); xlabel('time (s)'); title('True and estimated mass'); legend('Location','best');

    subplot(2,2,2); hold on; grid on;
    plot(time,runs(1).gravity_residual_true,'k--','LineWidth',1.8,'DisplayName','true');
    for j = 1:number_of_runs
        plot(time,runs(j).gravity_residual_hat,'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    ylabel('\beta_g (m/s^2)'); xlabel('time (s)'); title('Gravity-trim residual'); legend('Location','best');

    subplot(2,2,3); hold on; grid on;
    plot(time,runs(1).mass_true*p.g,'k--','LineWidth',1.8,'DisplayName','true mg');
    for j = 1:number_of_runs
        plot(time,runs(j).trim_thrust_used,'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    ylabel('trim thrust (N)'); xlabel('time (s)'); title('Estimated/used trim thrust'); legend('Location','best');

    subplot(2,2,4); hold on; grid on;
    for j = 1:number_of_runs
        plot(time,runs(j).lambda,'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    ylabel('\lambda_k'); xlabel('time (s)'); title('Common forgetting factor'); legend('Location','best');
    save_figure(fig,'case1_estimates',p);

    plot_position(runs,reference,time,colors,labels,'case1_position',p);
    plot_attitude(runs,reference,time,colors,labels,'case1_attitude',p);
    plot_inputs(runs,time,colors,labels,'case1_inputs',p);
    plot_prediction_errors(runs,time,colors,labels,'case1_prediction_errors',p);
    plot_qp_times(runs,time,colors,labels,'case1_qp_times',p);

elseif strcmpi(case_name,'helix')
    fig = new_figure('Case 2 trajectory tracking',p);
    subplot(1,2,1); hold on; grid on; axis equal;
    plot3(reference(1,:),reference(2,:),reference(3,:),'k--','LineWidth',1.8,'DisplayName','reference');
    for j = 1:number_of_runs
        plot3(runs(j).x(1,:),runs(j).x(2,:),runs(j).x(3,:), ...
            'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    xlabel('p_1 (m)'); ylabel('p_2 (m)'); zlabel('p_3 (m)');
    title('Aggressive helix'); view(36,24); legend('Location','best');

    subplot(1,2,2); hold on; grid on;
    for j = 1:number_of_runs
        error_norm = sqrt(sum((runs(j).x(1:3,:)-reference(1:3,:)).^2,1));
        plot(time,error_norm,'Color',colors(j,:),'LineWidth',1.2,'DisplayName',labels{j});
    end
    xlabel('time (s)'); ylabel('position error norm (m)');
    title('Trajectory tracking error'); legend('Location','best');
    save_figure(fig,'case2_trajectory',p);

    plot_attitude(runs,reference,time,colors,labels,'case2_attitude',p);
    plot_inputs(runs,time,colors,labels,'case2_inputs',p);
    plot_prediction_errors(runs,time,colors,labels,'case2_prediction_errors',p);
    plot_qp_times(runs,time,colors,labels,'case2_qp_times',p);
else
    error('plot_results:Case','Unknown case "%s".',case_name);
end
end

function plot_position(runs,reference,time,colors,labels,file_name,p)
fig = new_figure('Position tracking',p);
axis_names = {'p_1 (m)','p_2 (m)','p_3 (m)'};
for axis_index = 1:3
    subplot(3,1,axis_index); hold on; grid on;
    plot(time,reference(axis_index,:),'k--','LineWidth',1.6,'DisplayName','reference');
    for j = 1:numel(runs)
        plot(time,runs(j).x(axis_index,:),'Color',colors(j,:),'LineWidth',1.1,'DisplayName',labels{j});
    end
    ylabel(axis_names{axis_index});
    if axis_index == 1, title('Position tracking'); end
    if axis_index == 3, xlabel('time (s)'); end
end
legend('Location','best');
save_figure(fig,file_name,p);
end

function plot_attitude(runs,reference,time,colors,labels,file_name,p)
fig = new_figure('Attitude',p);
axis_names = {'yaw \psi (deg)','roll \phi (deg)','pitch \theta (deg)'};
for axis_index = 1:3
    subplot(3,1,axis_index); hold on; grid on;
    plot(time,reference(axis_index+3,:)*180/pi,'k--','LineWidth',1.4,'DisplayName','reference');
    for j = 1:numel(runs)
        plot(time,runs(j).x(axis_index+3,:)*180/pi,'Color',colors(j,:),'LineWidth',1.1,'DisplayName',labels{j});
    end
    ylabel(axis_names{axis_index});
    if axis_index == 1, title('Intrinsic Z-X-Y Euler angles'); end
    if axis_index == 3, xlabel('time (s)'); end
end
legend('Location','best');
save_figure(fig,file_name,p);
end

function plot_inputs(runs,time,colors,labels,file_name,p)
fig = new_figure('Inputs',p);
axis_names = {'thrust f (N)','torque \tau_1 (N m)','torque \tau_2 (N m)','torque \tau_3 (N m)'};
for input_index = 1:4
    subplot(4,1,input_index); hold on; grid on;
    for j = 1:numel(runs)
        stairs(time,runs(j).u(input_index,:),'Color',colors(j,:),'LineWidth',1.0,'DisplayName',labels{j});
    end
    yline(p.u_min(input_index),'k:','HandleVisibility','off');
    yline(p.u_max(input_index),'k:','HandleVisibility','off');
    ylabel(axis_names{input_index});
    if input_index == 1, title('Input histories and actuator limits'); end
    if input_index == 4, xlabel('time (s)'); end
end
legend('Location','best');
save_figure(fig,file_name,p);
end

function plot_prediction_errors(runs,time,colors,labels,file_name,p)
fig = new_figure('One-step prediction errors',p);
groups = {1:3,4:6,7:9,10:12};
group_names = {'position','attitude','velocity','angular rate'};
for group_index = 1:4
    subplot(4,1,group_index); hold on; grid on;
    for j = 1:numel(runs)
        value = sqrt(sum(runs(j).prediction_error(groups{group_index},:).^2,1));
        semilogy(time,max(value,1.0e-12),'Color',colors(j,:),'LineWidth',1.0,'DisplayName',labels{j});
    end
    ylabel(sprintf('%s norm',group_names{group_index}));
    if group_index == 1, title('A-priori one-step prediction errors'); end
    if group_index == 4, xlabel('time (s)'); end
end
legend('Location','best');
save_figure(fig,file_name,p);
end

function plot_qp_times(runs,time,colors,labels,file_name,p)
fig = new_figure('QP solution times',p);
hold on; grid on;
for j = 1:numel(runs)
    plot(time,1e3*runs(j).qp_time,'Color',colors(j,:),'LineWidth',1.0,'DisplayName',labels{j});
end
xlabel('time (s)'); ylabel('quadprog time (ms)');
title('QP computation time'); legend('Location','best');
save_figure(fig,file_name,p);
end

function fig = new_figure(name,p)
fig = figure('Name',name,'Color','w','Visible',p.graphics.visible);
set(fig,'Position',[100 100 1050 700]);
end

function save_figure(fig,file_name,p)
if ~p.graphics.save_figures
    return
end
if ~exist(p.output.directory,'dir')
    mkdir(p.output.directory);
end
saveas(fig,fullfile(p.output.directory,[file_name '.png']));
if p.graphics.save_matlab_figures
    savefig(fig,fullfile(p.output.directory,[file_name '.fig']));
end
end
