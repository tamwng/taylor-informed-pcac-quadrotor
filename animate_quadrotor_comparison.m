function video_file = animate_quadrotor_comparison(results,p,file_stem)
%ANIMATE_QUADROTOR_COMPARISON
% Animate D=1, D=2, and D=3 trajectories with one moving D=3 vehicle.

if nargin < 3
    file_stem = 'quadrotor_comparison';
end

if numel(results) < 3
    error('results must contain at least the D=1, D=2, and D=3 runs.');
end

if ~exist(p.output.directory,'dir')
    mkdir(p.output.directory);
end

video_file = fullfile(p.output.directory,[file_stem '.mp4']);

try
    writer = VideoWriter(video_file,'MPEG-4');
catch
    video_file = fullfile(p.output.directory,[file_stem '.avi']);
    writer = VideoWriter(video_file,'Motion JPEG AVI');
end

fps = p.graphics.animation_fps;
writer.FrameRate = fps;
open(writer);

cleanup = onCleanup(@() close_writer(writer));

%% Prepare data

number_of_runs = 3;

for j = 1:number_of_runs

    t = results{j}.time(:);
    number_of_samples = numel(t);

    if size(results{j}.x,2) == number_of_samples
        x = results{j}.x.';
    elseif size(results{j}.x,1) == number_of_samples
        x = results{j}.x;
    else
        error('State dimensions are inconsistent with the time vector.');
    end

    if size(results{j}.reference,2) == number_of_samples
        reference = results{j}.reference.';
    elseif size(results{j}.reference,1) == number_of_samples
        reference = results{j}.reference;
    else
        error('Reference dimensions are inconsistent with the time vector.');
    end

    time_data{j} = t;
    state_data{j} = x;
    reference_data{j} = reference;
end

t_start = max(cellfun(@(x) x(1),time_data));
t_end = min(cellfun(@(x) x(end),time_data));

t_video = (t_start:1/fps:t_end).';

if t_video(end) < t_end
    t_video(end+1,1) = t_end;
end

%% Interpolate all runs

state_video = cell(1,number_of_runs);

for j = 1:number_of_runs

    t = time_data{j};
    x = state_data{j};

    x_interp = zeros(numel(t_video),12);

    x_interp(:,1:3) = interp1( ...
        t,x(:,1:3),t_video,'pchip');

    angles = unwrap(x(:,4:6),[],1);

    x_interp(:,4:6) = interp1( ...
        t,angles,t_video,'pchip');

    x_interp(:,7:12) = interp1( ...
        t,x(:,7:12),t_video,'pchip');

    state_video{j} = x_interp;
end

reference_video = interp1( ...
    time_data{1}, ...
    reference_data{1}(:,1:3), ...
    t_video, ...
    'pchip');

%% Axis limits

all_position = reference_video;

for j = 1:number_of_runs
    all_position = [all_position; state_video{j}(:,1:3)];
end

minimum = min(all_position,[],1).' - 0.8;
maximum = max(all_position,[],1).' + 0.8;

span = max(maximum-minimum);

if span <= 0
    span = 1;
end

center = 0.5*(minimum+maximum);

minimum = center-0.55*span;
maximum = center+0.55*span;

%% Quadrotor geometry

arm = p.graphics.arm_length;

body_points = [ ...
     arm, -arm,    0,    0; ...
        0,    0,  arm, -arm; ...
        0,    0,    0,    0];

%% Figure

fig = figure( ...
    'Name','Quadrotor comparison', ...
    'Color','w', ...
    'Visible',p.graphics.visible, ...
    'Units','pixels', ...
    'Position',[100 100 900 720], ...
    'Resize','off', ...
    'Renderer','opengl');

drawnow;

axes_handle = axes(fig);

hold(axes_handle,'on');
grid(axes_handle,'on');
axis(axes_handle,'equal');
axis(axes_handle,'manual');

xlim(axes_handle,[minimum(1),maximum(1)]);
ylim(axes_handle,[minimum(2),maximum(2)]);
zlim(axes_handle,[minimum(3),maximum(3)]);

xlabel(axes_handle,'p_1 (m)');
ylabel(axes_handle,'p_2 (m)');
zlabel(axes_handle,'p_3 (m)');

view(axes_handle,36,24);

%% Reference and trajectory handles

reference_handle = plot3( ...
    axes_handle, ...
    reference_video(:,1), ...
    reference_video(:,2), ...
    reference_video(:,3), ...
    'k--', ...
    'LineWidth',1.3, ...
    'DisplayName','Reference');

d1_handle = plot3( ...
    axes_handle,nan,nan,nan, ...
    'b-', ...
    'LineWidth',1.6, ...
    'DisplayName','D = 1');

d2_handle = plot3( ...
    axes_handle,nan,nan,nan, ...
    'g-', ...
    'LineWidth',1.6, ...
    'DisplayName','D = 2');

d3_handle = plot3( ...
    axes_handle,nan,nan,nan, ...
    'r-', ...
    'LineWidth',1.8, ...
    'DisplayName','D = 3');

%% Moving D=3 quadrotor

arm_1_handle = plot3( ...
    axes_handle,nan(1,2),nan(1,2),nan(1,2), ...
    'r-', ...
    'LineWidth',4, ...
    'HandleVisibility','off');

arm_2_handle = plot3( ...
    axes_handle,nan(1,2),nan(1,2),nan(1,2), ...
    'k-', ...
    'LineWidth',4, ...
    'HandleVisibility','off');

rotor_handle = scatter3( ...
    axes_handle, ...
    nan(1,4),nan(1,4),nan(1,4), ...
    70, ...
    'filled', ...
    'MarkerFaceColor',[0.2 0.2 0.2], ...
    'HandleVisibility','off');

body_axis_handle = quiver3( ...
    axes_handle, ...
    0,0,0, ...
    0,0,0, ...
    0, ...
    'k', ...
    'LineWidth',1.5, ...
    'MaxHeadSize',0.8, ...
    'HandleVisibility','off');

legend( ...
    axes_handle, ...
    [reference_handle,d1_handle,d2_handle,d3_handle], ...
    'Location','best');

title_handle = title( ...
    axes_handle, ...
    sprintf('Taylor-informed PCAC comparison, t = %.2f s',t_video(1)));

%% Write frames

target_height = [];
target_width = [];

for k = 1:numel(t_video)

    x1 = state_video{1};
    x2 = state_video{2};
    x3 = state_video{3};

    set(d1_handle, ...
        'XData',x1(1:k,1), ...
        'YData',x1(1:k,2), ...
        'ZData',x1(1:k,3));

    set(d2_handle, ...
        'XData',x2(1:k,1), ...
        'YData',x2(1:k,2), ...
        'ZData',x2(1:k,3));

    set(d3_handle, ...
        'XData',x3(1:k,1), ...
        'YData',x3(1:k,2), ...
        'ZData',x3(1:k,3));

    position = x3(k,1:3).';
    attitude = x3(k,4:6).';

    R = intrinsic_zxy_rotation(attitude);
    points = position + R*body_points;

    set(arm_1_handle, ...
        'XData',points(1,1:2), ...
        'YData',points(2,1:2), ...
        'ZData',points(3,1:2));

    set(arm_2_handle, ...
        'XData',points(1,3:4), ...
        'YData',points(2,3:4), ...
        'ZData',points(3,3:4));

    set(rotor_handle, ...
        'XData',points(1,:), ...
        'YData',points(2,:), ...
        'ZData',points(3,:));

    body_z = R(:,3);

    set(body_axis_handle, ...
        'XData',position(1), ...
        'YData',position(2), ...
        'ZData',position(3), ...
        'UData',0.55*body_z(1), ...
        'VData',0.55*body_z(2), ...
        'WData',0.55*body_z(3));

    set(title_handle, ...
        'String',sprintf( ...
        'Taylor-informed PCAC comparison, t = %.2f s', ...
        t_video(k)));

    drawnow;

    frame = getframe(fig);
    image_data = frame.cdata;

    if isempty(target_height)

        target_height = 2*floor(size(image_data,1)/2);
        target_width = 2*floor(size(image_data,2)/2);

    end

    image_data = force_image_size( ...
        image_data,target_height,target_width);

    frame.cdata = image_data;
    frame.colormap = [];

    writeVideo(writer,frame);
end

close(writer);
delete(cleanup);

end

function R = intrinsic_zxy_rotation(xi)
%INTRINSIC_ZXY_ROTATION R = Rz(psi)Rx(phi)Ry(theta).

psi = xi(1);
phi = xi(2);
theta = xi(3);

cpsi = cos(psi);
spsi = sin(psi);
cphi = cos(phi);
sphi = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);

R = [ ...
    cpsi*ctheta-sphi*spsi*stheta, ...
    -spsi*cphi, ...
    cpsi*stheta+sphi*spsi*ctheta; ...
    spsi*ctheta+sphi*cpsi*stheta, ...
    cpsi*cphi, ...
    spsi*stheta-sphi*cpsi*ctheta; ...
    -cphi*stheta, ...
    sphi, ...
    cphi*ctheta];

end

function image_out = force_image_size( ...
    image_in,target_height,target_width)
%FORCE_IMAGE_SIZE Crop or pad an image to fixed dimensions.

current_height = size(image_in,1);
current_width = size(image_in,2);
number_of_channels = size(image_in,3);

image_out = zeros( ...
    target_height, ...
    target_width, ...
    number_of_channels, ...
    'like',image_in);

copy_height = min(current_height,target_height);
copy_width = min(current_width,target_width);

source_row_start = floor((current_height-copy_height)/2)+1;
source_col_start = floor((current_width-copy_width)/2)+1;

target_row_start = floor((target_height-copy_height)/2)+1;
target_col_start = floor((target_width-copy_width)/2)+1;

image_out( ...
    target_row_start:target_row_start+copy_height-1, ...
    target_col_start:target_col_start+copy_width-1,:) = ...
    image_in( ...
    source_row_start:source_row_start+copy_height-1, ...
    source_col_start:source_col_start+copy_width-1,:);

end

function close_writer(writer)

try
    close(writer);
catch
end

end