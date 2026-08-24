function plot_results(run_collection,p,case_name)
%PLOT_RESULTS Publication-quality comparison figures for both cases.

%% USER-TUNABLE FIGURE SETTINGS
style.font_name = 'Times New Roman';
style.axes_font_size = 11;
style.label_font_size = 14;
style.legend_font_size = 11;
style.title_font_size = 14;
style.axes_line_width = 0.8;
style.data_line_width = 1.65;
style.reference_line_width = 1.85;
style.event_line_width = 1.15;

style.colors.reference = [0.00, 0.00, 0.00];
style.colors.D1 = [0.00, 0.35, 0.75];
style.colors.D2 = [0.10, 0.55, 0.20];
style.colors.D3 = [0.80, 0.10, 0.10];
style.colors.event = [0.35, 0.35, 0.35];
style.line_styles.reference = '--';
style.line_styles.D1 = '-';
style.line_styles.D2 = '-';
style.line_styles.D3 = '-';

style.legend_labels_tracking = {'Reference','$D=1$','$D=2$','$D=3$'};
style.legend_labels_estimates = {'True','$D=1$','$D=2$','$D=3$'};
style.legend_labels_degrees = {'$D=1$','$D=2$','$D=3$'};
style.legend_orientation = 'horizontal';
style.legend_box = 'off';
style.legend_columns_case1_tracking = 4;
style.legend_columns_case1_estimates = 4;
style.legend_columns_case2_trajectory = 4;
style.legend_columns_case2_detail = 3;
style.legend_location_case1_tracking = 'northoutside';
style.legend_location_case1_estimates = 'northoutside';
style.legend_location_case2_trajectory = 'northoutside';
style.legend_location_case2_detail = 'northoutside';

style.figure_position_case1_tracking = [80, 60, 820, 920];
style.figure_position_case1_estimates = [110, 80, 820, 740];
style.figure_position_case2_trajectory = [140, 100, 820, 650];
style.figure_position_case2_detail = [170, 120, 820, 740];
style.renderer = 'painters';
style.tile_spacing = 'compact';
style.padding = 'compact';

style.grid = 'on';
style.minor_grid = 'off';
style.grid_alpha = 0.18;
style.box = 'on';
style.show_titles = false;
style.titles.case1_tracking = 'Case 1: position tracking';
style.titles.case1_estimates = 'Case 1: online estimates and variable-rate forgetting';
style.titles.case2_trajectory = 'Case 2: aggressive helix tracking';
style.titles.case2_detail = 'Case 2: prediction and attitude';

style.position_labels = {'$p_1$ (m)','$p_2$ (m)','$p_3$ (m)'};
style.position_error_label = '$\|p-r_p\|_2$ (m)';
style.time_label = '$t$ (s)';
style.mass_label = '$m,\ \hat m$ (kg)';
style.trim_label = '$f_{\mathrm{trim}},\ \hat f_{\mathrm{trim}}$ (N)';
style.forgetting_label = '$\lambda_k$';
style.prediction_error_label = '$\|e_k\|_2$';
style.roll_label = '$\varphi$ (deg)';
style.pitch_label = '$\theta$ (deg)';
style.mass_change_label = '$\mathrm{Mass\ change}$';

style.mark_mass_change_on_tracking = true;
style.mark_mass_change_on_estimates = true;
style.mass_change_label_subplot = 1;
style.mass_change_label_vertical_alignment = 'top';
style.mass_change_label_horizontal_alignment = 'left';

style.prediction_error_states = 1:12;
style.prediction_error_scale = 'linear';    % 'linear' for paper Fig. 4; 'log' is optional
style.prediction_error_floor = 1.0e-10;
style.roll_state_index = 5;
style.pitch_state_index = 6;
style.angle_scale = 180/pi;
style.helix_view = [36,24];
style.helix_axis_equal = true;
style.vrf_ylim = [];                         % Example: [0.4,1.02]

style.save_pdf = true;
style.save_png = true;
style.png_resolution = 300;
style.close_after_saving = false;
style.file_names.case1_tracking = 'case1_tracking';
style.file_names.case1_estimates = 'case1_estimates_vrf';
style.file_names.case2_trajectory = 'case2_trajectory';
style.file_names.case2_detail = 'case2_prediction_attitude';

%% DATA SELECTION
runs = select_degree_runs(run_collection,[1,2,3]);
time = runs(1).time(:).';
reference = runs(1).reference;
validate_run_dimensions(runs,time,reference);

switch lower(case_name)
    case 'mass_change'
        plot_case1_tracking(runs,time,reference,p,style);
        plot_case1_estimates(runs,time,p,style);

    case 'helix'
        plot_case2_trajectory(runs,reference,p,style);
        plot_case2_detail(runs,time,p,style);

    otherwise
        error('plot_results:Case','Unknown case "%s".',case_name);
end
end

function plot_case1_tracking(runs,time,reference,p,style)
fig = create_figure('Case 1 position tracking', ...
    style.figure_position_case1_tracking,p,style);
tiles = tiledlayout(fig,4,1, ...
    'TileSpacing',style.tile_spacing,'Padding',style.padding);
axes_list = gobjects(4,1);
legend_handles = gobjects(4,1);

for state_index = 1:3
    ax = nexttile(tiles);
    axes_list(state_index) = ax;
    hold(ax,'on');

    h_ref = plot(ax,time,reference(state_index,:), ...
        'Color',style.colors.reference, ...
        'LineStyle',style.line_styles.reference, ...
        'LineWidth',style.reference_line_width);

    h_runs = plot_degree_curves(ax,time,runs, ...
        @(r) r.x(state_index,:),style);

    if state_index == 1
        legend_handles = [h_ref;h_runs(:)];
    end

    ylabel(ax,style.position_labels{state_index}, ...
        'Interpreter','latex','FontSize',style.label_font_size);
    apply_axis_style(ax,style);
    xlim(ax,[time(1),time(end)]);
    hide_x_tick_labels(ax);

    if style.mark_mass_change_on_tracking
        add_mass_change_line(ax,p.case1.mass_change_time, ...
            state_index == style.mass_change_label_subplot,style);
    end
end

ax = nexttile(tiles);
axes_list(4) = ax;
hold(ax,'on');
for j = 1:3
    error_norm = vecnorm(runs(j).x(1:3,:)-reference(1:3,:),2,1);
    plot(ax,time,error_norm, ...
        'Color',degree_color(style,runs(j).D), ...
        'LineStyle',degree_line_style(style,runs(j).D), ...
        'LineWidth',style.data_line_width);
end
ylabel(ax,style.position_error_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
xlabel(ax,style.time_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);

if style.mark_mass_change_on_tracking
    add_mass_change_line(ax,p.case1.mass_change_time,false,style);
end

linkaxes(axes_list,'x');
add_legend(axes_list(1),legend_handles,style.legend_labels_tracking, ...
    style.legend_location_case1_tracking, ...
    style.legend_columns_case1_tracking,style);

if style.show_titles
    title(axes_list(1),style.titles.case1_tracking, ...
        'Interpreter','latex','FontSize',style.title_font_size);
end

save_figure(fig,style.file_names.case1_tracking,p,style);
end

function plot_case1_estimates(runs,time,p,style)
fig = create_figure('Case 1 estimates', ...
    style.figure_position_case1_estimates,p,style);
tiles = tiledlayout(fig,3,1, ...
    'TileSpacing',style.tile_spacing,'Padding',style.padding);
axes_list = gobjects(3,1);

% Mass estimate
ax = nexttile(tiles);
axes_list(1) = ax;
hold(ax,'on');
h_true = plot(ax,time,runs(1).mass_true, ...
    'Color',style.colors.reference, ...
    'LineStyle',style.line_styles.reference, ...
    'LineWidth',style.reference_line_width);
h_runs = plot_degree_curves(ax,time,runs,@(r) r.mass_hat,style);
ylabel(ax,style.mass_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);
hide_x_tick_labels(ax);
if style.mark_mass_change_on_estimates
    add_mass_change_line(ax,p.case1.mass_change_time, ...
        style.mass_change_label_subplot == 1,style);
end

% Trim-thrust estimate
ax = nexttile(tiles);
axes_list(2) = ax;
hold(ax,'on');
plot(ax,time,runs(1).mass_true*p.g, ...
    'Color',style.colors.reference, ...
    'LineStyle',style.line_styles.reference, ...
    'LineWidth',style.reference_line_width);
plot_degree_curves(ax,time,runs,@(r) r.trim_thrust_hat,style);
ylabel(ax,style.trim_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);
hide_x_tick_labels(ax);
if style.mark_mass_change_on_estimates
    add_mass_change_line(ax,p.case1.mass_change_time, ...
        style.mass_change_label_subplot == 2,style);
end

% Variable-rate forgetting factor
ax = nexttile(tiles);
axes_list(3) = ax;
hold(ax,'on');
plot_degree_curves(ax,time,runs,@(r) r.lambda,style);
ylabel(ax,style.forgetting_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
xlabel(ax,style.time_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);
if ~isempty(style.vrf_ylim)
    ylim(ax,style.vrf_ylim);
end
if style.mark_mass_change_on_estimates
    add_mass_change_line(ax,p.case1.mass_change_time, ...
        style.mass_change_label_subplot == 3,style);
end

linkaxes(axes_list,'x');
add_legend(axes_list(1),[h_true;h_runs(:)],style.legend_labels_estimates, ...
    style.legend_location_case1_estimates, ...
    style.legend_columns_case1_estimates,style);

if style.show_titles
    title(axes_list(1),style.titles.case1_estimates, ...
        'Interpreter','latex','FontSize',style.title_font_size);
end

save_figure(fig,style.file_names.case1_estimates,p,style);
end

function plot_case2_trajectory(runs,reference,p,style)
fig = create_figure('Case 2 trajectory', ...
    style.figure_position_case2_trajectory,p,style);
ax = axes(fig);
hold(ax,'on');

h_ref = plot3(ax,reference(1,:),reference(2,:),reference(3,:), ...
    'Color',style.colors.reference, ...
    'LineStyle',style.line_styles.reference, ...
    'LineWidth',style.reference_line_width);

h_runs = gobjects(3,1);
for j = 1:3
    h_runs(j) = plot3(ax,runs(j).x(1,:),runs(j).x(2,:),runs(j).x(3,:), ...
        'Color',degree_color(style,runs(j).D), ...
        'LineStyle',degree_line_style(style,runs(j).D), ...
        'LineWidth',style.data_line_width);
end

xlabel(ax,'$p_1$ (m)','Interpreter','latex','FontSize',style.label_font_size);
ylabel(ax,'$p_2$ (m)','Interpreter','latex','FontSize',style.label_font_size);
zlabel(ax,'$p_3$ (m)','Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
view(ax,style.helix_view(1),style.helix_view(2));
axis(ax,'tight');
if style.helix_axis_equal
    axis(ax,'equal');
end

add_legend(ax,[h_ref;h_runs],style.legend_labels_tracking, ...
    style.legend_location_case2_trajectory, ...
    style.legend_columns_case2_trajectory,style);

if style.show_titles
    title(ax,style.titles.case2_trajectory, ...
        'Interpreter','latex','FontSize',style.title_font_size);
end

save_figure(fig,style.file_names.case2_trajectory,p,style);
end

function plot_case2_detail(runs,time,p,style)
fig = create_figure('Case 2 prediction and attitude', ...
    style.figure_position_case2_detail,p,style);
tiles = tiledlayout(fig,3,1, ...
    'TileSpacing',style.tile_spacing,'Padding',style.padding);
axes_list = gobjects(3,1);

% One-step prediction error
ax = nexttile(tiles);
axes_list(1) = ax;
hold(ax,'on');
h_error = gobjects(3,1);
for j = 1:3
    error_norm = prediction_error_norm( ...
        runs(j).prediction_error,style.prediction_error_states, ...
        style.prediction_error_floor);

    if strcmpi(style.prediction_error_scale,'log')
        h_error(j) = semilogy(ax,time,error_norm, ...
            'Color',degree_color(style,runs(j).D), ...
            'LineStyle',degree_line_style(style,runs(j).D), ...
            'LineWidth',style.data_line_width);
    else
        h_error(j) = plot(ax,time,error_norm, ...
            'Color',degree_color(style,runs(j).D), ...
            'LineStyle',degree_line_style(style,runs(j).D), ...
            'LineWidth',style.data_line_width);
    end
end
set(ax,'YScale',style.prediction_error_scale);
ylabel(ax,style.prediction_error_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);
hide_x_tick_labels(ax);

% Roll
ax = nexttile(tiles);
axes_list(2) = ax;
hold(ax,'on');
plot_degree_curves(ax,time,runs, ...
    @(r) style.angle_scale*r.x(style.roll_state_index,:),style);
ylabel(ax,style.roll_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);
hide_x_tick_labels(ax);

% Pitch
ax = nexttile(tiles);
axes_list(3) = ax;
hold(ax,'on');
plot_degree_curves(ax,time,runs, ...
    @(r) style.angle_scale*r.x(style.pitch_state_index,:),style);
ylabel(ax,style.pitch_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
xlabel(ax,style.time_label, ...
    'Interpreter','latex','FontSize',style.label_font_size);
apply_axis_style(ax,style);
xlim(ax,[time(1),time(end)]);

linkaxes(axes_list,'x');
add_legend(axes_list(1),h_error,style.legend_labels_degrees, ...
    style.legend_location_case2_detail, ...
    style.legend_columns_case2_detail,style);

if style.show_titles
    title(axes_list(1),style.titles.case2_detail, ...
        'Interpreter','latex','FontSize',style.title_font_size);
end

save_figure(fig,style.file_names.case2_detail,p,style);
end

function handles = plot_degree_curves(ax,time,runs,value_function,style)
handles = gobjects(3,1);
for j = 1:3
    handles(j) = plot(ax,time,value_function(runs(j)), ...
        'Color',degree_color(style,runs(j).D), ...
        'LineStyle',degree_line_style(style,runs(j).D), ...
        'LineWidth',style.data_line_width);
end
end

function value = prediction_error_norm(prediction_error,state_indices,error_floor)
selected_error = prediction_error(state_indices,:);
valid = all(isfinite(selected_error),1);
value = nan(1,size(selected_error,2));
value(valid) = vecnorm(selected_error(:,valid),2,1);
value(valid) = max(value(valid),error_floor);
end

function add_mass_change_line(ax,event_time,show_label,style)
if show_label
    event_line = xline(ax,event_time,'--',style.mass_change_label, ...
        'Color',style.colors.event, ...
        'LineWidth',style.event_line_width, ...
        'Interpreter','latex', ...
        'LabelVerticalAlignment',style.mass_change_label_vertical_alignment, ...
        'LabelHorizontalAlignment',style.mass_change_label_horizontal_alignment, ...
        'HandleVisibility','off');
    event_line.FontSize = style.legend_font_size;
else
    xline(ax,event_time,'--', ...
        'Color',style.colors.event, ...
        'LineWidth',style.event_line_width, ...
        'HandleVisibility','off');
end
end

function apply_axis_style(ax,style)
set(ax, ...
    'FontName',style.font_name, ...
    'FontSize',style.axes_font_size, ...
    'TickLabelInterpreter','latex', ...
    'LineWidth',style.axes_line_width, ...
    'Box',style.box, ...
    'Layer','top', ...
    'GridAlpha',style.grid_alpha, ...
    'XMinorGrid',style.minor_grid, ...
    'YMinorGrid',style.minor_grid);
grid(ax,style.grid);
end

function hide_x_tick_labels(ax)
ax.XTickLabel = [];
end

function add_legend(ax,handles,labels,location,num_columns,style)
legend_handle = legend(ax,handles,labels, ...
    'Interpreter','latex', ...
    'FontSize',style.legend_font_size, ...
    'Location',location, ...
    'Orientation',style.legend_orientation, ...
    'NumColumns',num_columns);
legend_handle.Box = style.legend_box;
end

function fig = create_figure(name,position,p,style)
visibility = 'on';
if isfield(p,'graphics') && isfield(p.graphics,'visible')
    visibility = p.graphics.visible;
end

fig = figure( ...
    'Name',name, ...
    'Color','w', ...
    'Visible',visibility, ...
    'Units','pixels', ...
    'Position',position, ...
    'Renderer',style.renderer);
end

function save_figure(fig,file_name,p,style)
if ~isfield(p,'graphics') || ~isfield(p.graphics,'save_figures') || ...
        ~p.graphics.save_figures
    return
end

if ~exist(p.output.directory,'dir')
    mkdir(p.output.directory);
end

drawnow;

if style.save_pdf
    pdf_file = fullfile(p.output.directory,[file_name '.pdf']);
    try
        exportgraphics(fig,pdf_file, ...
            'ContentType','vector','BackgroundColor','white');
    catch
        print(fig,pdf_file,'-dpdf','-vector','-bestfit');
    end
end

if style.save_png
    png_file = fullfile(p.output.directory,[file_name '.png']);
    try
        exportgraphics(fig,png_file, ...
            'Resolution',style.png_resolution,'BackgroundColor','white');
    catch
        print(fig,png_file,'-dpng',sprintf('-r%d',style.png_resolution));
    end
end

if isfield(p.graphics,'save_matlab_figures') && ...
        p.graphics.save_matlab_figures
    savefig(fig,fullfile(p.output.directory,[file_name '.fig']));
end

if style.close_after_saving
    close(fig);
end
end

function runs = select_degree_runs(run_collection,degrees)
if iscell(run_collection)
    all_runs = [run_collection{:}];
else
    all_runs = run_collection;
end

if isempty(all_runs)
    error('plot_results:Empty','No simulation results were supplied.');
end

if isfield(all_runs,'fixed_trim')
    all_runs = all_runs(~[all_runs.fixed_trim]);
end

runs = repmat(all_runs(1),1,numel(degrees));
for j = 1:numel(degrees)
    index = find([all_runs.D] == degrees(j),1,'first');
    if isempty(index)
        error('plot_results:Degree', ...
            'No non-fixed-trim result was found for D=%d.',degrees(j));
    end
    runs(j) = all_runs(index);
end
end

function validate_run_dimensions(runs,time,reference)
number_of_samples = numel(time);
if size(reference,2) ~= number_of_samples
    error('plot_results:Reference', ...
        'The reference and time vectors have inconsistent dimensions.');
end

for j = 1:numel(runs)
    if numel(runs(j).time) ~= number_of_samples || ...
            size(runs(j).x,2) ~= number_of_samples || ...
            size(runs(j).prediction_error,2) ~= number_of_samples
        error('plot_results:Dimensions', ...
            'Run D=%d is inconsistent with the common time vector.',runs(j).D);
    end
end
end

function color = degree_color(style,D)
switch D
    case 1
        color = style.colors.D1;
    case 2
        color = style.colors.D2;
    case 3
        color = style.colors.D3;
    otherwise
        error('plot_results:DegreeColor','Unsupported Taylor degree D=%d.',D);
end
end

function line_style = degree_line_style(style,D)
switch D
    case 1
        line_style = style.line_styles.D1;
    case 2
        line_style = style.line_styles.D2;
    case 3
        line_style = style.line_styles.D3;
    otherwise
        error('plot_results:DegreeStyle','Unsupported Taylor degree D=%d.',D);
end
end
