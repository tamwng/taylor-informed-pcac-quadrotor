function reference = generate_reference(case_name, time, p)
%GENERATE_REFERENCE Full-state references for both cases.

time = time(:).';
number_of_samples = numel(time);
reference = zeros(12,number_of_samples);

switch lower(case_name)
    case 'mass_change'
        a = p.case1.reference;
        [s,sd,~] = smooth_ramp(time,a.ramp_time);

        sx = sin(a.frequency_xy(1)*time);
        cx = cos(a.frequency_xy(1)*time);
        sy = sin(a.frequency_xy(2)*time);
        cy = cos(a.frequency_xy(2)*time);
        sz = sin(a.z_frequency*time);
        cz = cos(a.z_frequency*time);

        position = [a.amplitude_xy(1)*s.*sx; ...
                    a.amplitude_xy(2)*s.*sy; ...
                    s.*(a.z_offset + a.z_amplitude*sz)];
        velocity = [a.amplitude_xy(1)*(sd.*sx + s*a.frequency_xy(1).*cx); ...
                    a.amplitude_xy(2)*(sd.*sy + s*a.frequency_xy(2).*cy); ...
                    sd.*(a.z_offset + a.z_amplitude*sz) + ...
                    s*a.z_amplitude*a.z_frequency.*cz];

    case 'helix'
        a = p.case2.reference;
        [s,sd,integral_s] = smooth_ramp(time,a.ramp_time);
        phase = a.angular_rate*(time-a.ramp_time);
        cp = cos(phase);
        sp = sin(phase);
        sz = sin(a.z_frequency*time);
        cz = cos(a.z_frequency*time);

        position = [a.radius*s.*(cp-1); ...
                    a.radius*s.*sp; ...
                    a.z_offset*s + a.climb_rate*integral_s + a.z_amplitude*s.*sz];
        velocity = [a.radius*(sd.*(cp-1) - s*a.angular_rate.*sp); ...
                    a.radius*(sd.*sp + s*a.angular_rate.*cp); ...
                    a.z_offset*sd + a.climb_rate*s + ...
                    a.z_amplitude*(sd.*sz + s*a.z_frequency.*cz)];

    otherwise
        error('generate_reference:Case','Unknown case "%s".',case_name);
end

reference(1:3,:) = position;
reference(7:9,:) = velocity;
end

function [s,sd,integral_s] = smooth_ramp(time,ramp_time)
s = zeros(size(time));
sd = zeros(size(time));
integral_s = zeros(size(time));

positive = time > 0;
ramping = positive & time < ramp_time;
settled = time >= ramp_time;

s(ramping) = 0.5*(1-cos(pi*time(ramping)/ramp_time));
sd(ramping) = 0.5*pi/ramp_time*sin(pi*time(ramping)/ramp_time);
integral_s(ramping) = 0.5*time(ramping) - ...
    ramp_time/(2*pi)*sin(pi*time(ramping)/ramp_time);

s(settled) = 1;
integral_s(settled) = time(settled)-0.5*ramp_time;
end
