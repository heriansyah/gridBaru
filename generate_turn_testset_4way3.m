clear;
clc;

% ------------------------------------------------------------
% Generate 10 test scenarios for 4-way turns (left/straight/right).
% Output is only for evaluation of turn accuracy at intersection.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
OUT_CSV = fullfile(SCRIPT_DIR, 'test_intersection_4way3.csv');

DURATION_S = 40;
DT = 1.0;
TIMES = (0:DT:DURATION_S)';

APPROACH_LEN_M = 200;
TURN_RADIUS_HINT = 20;

% 100 scenarios (balanced across approaches/turns as much as possible).
N_SCENARIOS = 100;
base_cases = {
    'north', 'left';
    'north', 'straight';
    'north', 'right';
    'south', 'left';
    'south', 'straight';
    'south', 'right';
    'east',  'left';
    'east',  'straight';
    'east',  'right';
    'west',  'left';
    'west',  'straight';
    'west',  'right'
};

rng(123);
reps = ceil(N_SCENARIOS / size(base_cases, 1));
cases = repmat(base_cases, reps, 1);
idx = randperm(size(cases, 1));
cases = cases(idx, :);
cases = cases(1:N_SCENARIOS, :);

rows = cell(0, 4);
for i = 1:size(cases, 1)
    approach = cases{i, 1};
    maneuver = cases{i, 2};
    scenario = sprintf('Test_4way_%s_%s_%02d', approach, maneuver, i);
    xy = generate_route_4way(approach, maneuver, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, false);
    rows = append_rows(rows, scenario, TIMES, xy);
end

fid = fopen(OUT_CSV, 'w');
if fid < 0
    error('Cannot write %s', OUT_CSV);
end
fprintf(fid, 'scenario,timestamp,x,y\n');
for i = 1:size(rows, 1)
    fprintf(fid, '%s,%d,%.3f,%.3f\n', rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end
fclose(fid);

fprintf('Generated 10 test scenarios: %s\n', OUT_CSV);


function rows = append_rows(rows, scenario, times, xy)
    for k = 1:numel(times)
        rows(end+1, :) = {scenario, k, xy(k,1), xy(k,2)}; %#ok<AGROW>
    end
end

function xy = generate_route_4way(approach, maneuver, times, approach_len, turn_radius_hint, with_noise)
    center = [0, 0];

    switch lower(approach)
        case 'south'
            u_in = [0, 1];
            lane_offset = [1.75, 0];
        case 'north'
            u_in = [0, -1];
            lane_offset = [-1.75, 0];
        case 'west'
            u_in = [1, 0];
            lane_offset = [0, -1.75];
        case 'east'
            u_in = [-1, 0];
            lane_offset = [0, 1.75];
        otherwise
            error('Unknown approach: %s', approach);
    end

    p_start = center - u_in * approach_len + lane_offset;

    switch lower(maneuver)
        case 'straight'
            u_out = u_in;
        case 'left'
            u_out = [-u_in(2), u_in(1)];
        case 'right'
            u_out = [u_in(2), -u_in(1)];
        otherwise
            error('Unknown maneuver: %s', maneuver);
    end

    p_end = center + u_out * approach_len + lane_offset;

    p_in = center - u_in * turn_radius_hint + lane_offset;
    p_out = center + u_out * turn_radius_hint + lane_offset;

    if strcmpi(maneuver, 'straight')
        base = [linspace(p_start(1), p_end(1), 260)', linspace(p_start(2), p_end(2), 260)'];
    else
        seg1 = [linspace(p_start(1), p_in(1), 120)', linspace(p_start(2), p_in(2), 120)'];
        c1 = p_in + u_in * 10;
        c2 = p_out - u_out * 10;
        bez = bezier2d(p_in, c1, c2, p_out, 80);
        seg2 = [linspace(p_out(1), p_end(1), 120)', linspace(p_out(2), p_end(2), 120)'];
        base = [seg1; bez; seg2];
    end

    base = unique(base, 'rows', 'stable');
    s = [0; cumsum(sqrt(sum(diff(base).^2, 2)))];
    route_len = s(end);

    % Find closest point to center to anchor slowdown / lane shift.
    d_center = hypot(base(:,1), base(:,2));
    [~, i_center] = min(d_center);
    s_center = s(i_center);

    t = times(:);
    T = t(end);
    n = numel(t);
    s_lin = linspace(0, route_len, n)';

    v_nom = route_len / max(T, 1e-6);

    % Slowdown near intersection for turns.
    slow_amp = 0.35;
    slow_sigma = 25;
    if strcmpi(maneuver, 'straight')
        slow_profile = ones(n, 1);
    else
        slow_profile = 1 - slow_amp * exp(-((s_lin - s_center) / slow_sigma).^2);
    end

    if with_noise
        v_nom = v_nom * (1 + 0.08 * randn());
        v_nom = max(3.0, min(v_nom, 8.0));
        phase = 2*pi*rand();
        speed = v_nom * slow_profile .* (1 + 0.08 * sin(2*pi*t/T + phase));
    else
        speed = v_nom * slow_profile;
    end

    speed = max(0.8, speed);
    ds = speed * mean(diff(t));
    s_target = cumsum([0; ds(1:end-1)]);

    if s_target(end) > 0
        s_target = s_target * (route_len / s_target(end));
    end

    x = interp1(s, base(:,1), s_target, 'linear', 'extrap');
    y = interp1(s, base(:,2), s_target, 'linear', 'extrap');

    % Lateral drift for turns: shift to left/right lane near intersection.
    lane_shift = 1.2;  % meters
    shift_sigma = 30;
    if strcmpi(maneuver, 'left')
        shift_sign = 1.0;
    elseif strcmpi(maneuver, 'right')
        shift_sign = -1.0;
    else
        shift_sign = 0.0;
    end
    shift = shift_sign * lane_shift * (0.5 * (1 + tanh((s_target - s_center) / shift_sigma)));

    if abs(shift_sign) > 0
        gx = gradient(x);
        gy = gradient(y);
        gnorm = sqrt(gx.^2 + gy.^2) + 1e-9;
        nx = -gy ./ gnorm;
        ny = gx ./ gnorm;
        x = x + nx .* shift;
        y = y + ny .* shift;
    end

    if with_noise
        gx = gradient(x);
        gy = gradient(y);
        gnorm = sqrt(gx.^2 + gy.^2) + 1e-9;
        nx = -gy ./ gnorm;
        ny = gx ./ gnorm;
        jitter_scale = 0.10;
        if strcmpi(maneuver, 'straight')
            jitter_scale = 0.06;
        end
        jitter = jitter_scale * randn(size(x));
        x = x + nx .* jitter;
        y = y + ny .* jitter;
    end

    xy = [x, y];
end

function pts = bezier2d(p0, p1, p2, p3, n)
    u = linspace(0, 1, n)';
    pts = ((1-u).^3) * p0 + ...
          (3*(1-u).^2 .* u) * p1 + ...
          (3*(1-u) .* u.^2) * p2 + ...
          (u.^3) * p3;
end
