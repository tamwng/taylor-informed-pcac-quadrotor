function video_file = animate_quadrotor(result, p, file_stem)
%ANIMATE_QUADROTOR Save a 3-D animation using R=Rz(psi)Rx(phi)Ry(theta).

if nargin < 3
    file_stem = 'quadrotor_animation';
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
writer.FrameRate = p.graphics.animation_fps;
open(writer);
cleanup = onCleanup(@() close_writer(writer));

fig = figure('Name','Quadrotor animation','Color','w','Visible',p.graphics.visible);
set(fig,'Position',[100 100 900 720]);

all_position = [result.reference(1:3,:), result.x(1:3,:)];
minimum = min(all_position,[],2)-0.8;
maximum = max(all_position,[],2)+0.8;
span = max(maximum-minimum);
center = 0.5*(minimum+maximum);
minimum = center-0.55*span;
maximum = center+0.55*span;

arm = p.graphics.arm_length;
body_points = [arm, -arm, 0, 0; ...
               0, 0, arm, -arm; ...
               0, 0, 0, 0];

indices = 1:p.graphics.animation_stride:numel(result.time);
if indices(end) ~= numel(result.time)
    indices(end+1) = numel(result.time);
end

for k = indices
    clf(fig);
    axes_handle = axes(fig); %#ok<LAXES>
    hold(axes_handle,'on'); grid(axes_handle,'on'); axis(axes_handle,'equal');
    plot3(axes_handle,result.reference(1,:),result.reference(2,:),result.reference(3,:), ...
        'k--','LineWidth',1.1,'DisplayName','reference');
    plot3(axes_handle,result.x(1,1:k),result.x(2,1:k),result.x(3,1:k), ...
        'b-','LineWidth',1.5,'DisplayName',result.label);

    position = result.x(1:3,k);
    R = intrinsic_zxy_rotation(result.x(4:6,k));
    points = position + R*body_points;

    plot3(axes_handle,points(1,1:2),points(2,1:2),points(3,1:2), ...
        'r-','LineWidth',4,'HandleVisibility','off');
    plot3(axes_handle,points(1,3:4),points(2,3:4),points(3,3:4), ...
        'g-','LineWidth',4,'HandleVisibility','off');
    scatter3(axes_handle,points(1,:),points(2,:),points(3,:),70,'filled', ...
        'MarkerFaceColor',[0.2 0.2 0.2],'HandleVisibility','off');
    body_z = R(:,3);
    quiver3(axes_handle,position(1),position(2),position(3), ...
        0.55*body_z(1),0.55*body_z(2),0.55*body_z(3),0, ...
        'k','LineWidth',1.5,'MaxHeadSize',0.8,'HandleVisibility','off');

    xlim(axes_handle,[minimum(1),maximum(1)]);
    ylim(axes_handle,[minimum(2),maximum(2)]);
    zlim(axes_handle,[minimum(3),maximum(3)]);
    xlabel(axes_handle,'p_1 (m)'); ylabel(axes_handle,'p_2 (m)'); zlabel(axes_handle,'p_3 (m)');
    title(axes_handle,sprintf('%s, t = %.2f s',result.label,result.time(k)));
    view(axes_handle,36,24);
    legend(axes_handle,'Location','best');
    drawnow;
    writeVideo(writer,getframe(fig));
end

close(writer);
delete(cleanup);
end

function R = intrinsic_zxy_rotation(xi)
psi = xi(1); phi = xi(2); theta = xi(3);
cpsi = cos(psi); spsi = sin(psi);
cphi = cos(phi); sphi = sin(phi);
ctheta = cos(theta); stheta = sin(theta);
R = [cpsi*ctheta - sphi*spsi*stheta, -spsi*cphi, cpsi*stheta + sphi*spsi*ctheta; ...
     spsi*ctheta + sphi*cpsi*stheta,  cpsi*cphi, spsi*stheta - sphi*cpsi*ctheta; ...
     -cphi*stheta,                    sphi,      cphi*ctheta];
end

function close_writer(writer)
try
    close(writer);
catch
end
end
