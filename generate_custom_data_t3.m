clear;
clc;

% ------------------------------------------------------------
% SmartGrid3 synthetic generator for T-intersection
% - Grid size: 10m
% - Approach length to center: 200m (20 grids)
% - Behavior: slowdown near intersection + lateral drift on turns
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
OUT_CSV = fullfile(SCRIPT_DIR, 'custom_trajectories_t3.csv');
OUT_SUMMARY = fullfile(SCRIPT_DIR, 'custom_trajectories_t3_summary.txt');

DURATION_S = 50;
DT = 1.0;
TIMES = (0:DT:DURATION_S)';

APPROACH_LEN_M = 200;
TURN_RADIUS_HINT = 20;

% Per-approach maneuver distribution
N_TRAIN_PER_APPROACH = 12; % total train runs = 36
DIST = struct();
DIST.S = struct('left', 0.55, 'right', 0.45);
DIST.W = struct('straight', 0.70, 'right', 0.30);
DIST.E = struct('straight', 0.70, 'left', 0.30);

TEST_APPROACH = 'S';
TEST_MANEUVER = 'left';

approaches = {'S', 'W', 'E'};

fprintf('=== GENERATE SYNTHETIC T DATA (SMARTGRID3) ===\n');
fprintf('Duration: %d s, dt: %.1f s\n', DURATION_S, DT);
fprintf('Approach length: %.1f m (~%.1f grids)\n', APPROACH_LEN_M, APPROACH_LEN_M/10);

rows = cell(0, 4);
run_id = 0;

for a = 1:numel(approaches)
    approach = approaches{a};
    maneuvers = build_maneuvers_t(approach, N_TRAIN_PER_APPROACH, DIST);
    rng(200 + a);
    maneuvers = maneuvers(randperm(numel(maneuvers)));

    for i = 1:numel(maneuvers)
        run_id = run_id + 1; %#ok<NASGU>
        mv = maneuvers{i};
        [x, y] = generate_route_t(approach, mv, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, true);
        scenario = sprintf('Train_t_%s_Run_%03d_%s', approach, i, mv);
        rows = append_rows(rows, scenario, TIMES, [x, y]);
    end
end

[x_test, y_test] = generate_route_t(TEST_APPROACH, TEST_MANEUVER, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, false);
rows = append_rows(rows, sprintf('Test_t_%s_to_%s', TEST_APPROACH, TEST_MANEUVER), TIMES, [x_test, y_test]);

% Write custom CSV
fid = fopen(OUT_CSV, 'w');
if fid < 0
    error('Cannot write output file: %s', OUT_CSV);
end
fprintf(fid, 'scenario,timestamp,x,y\n');
for i = 1:size(rows, 1)
    fprintf(fid, '%s,%d,%.3f,%.3f\n', rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end
fclose(fid);

% Write summary
fid = fopen(OUT_SUMMARY, 'w');
if fid > 0
    fprintf(fid, 'SmartGrid3 synthetic T-generation summary\n');
    fprintf(fid, 'Duration(s): %d\n', DURATION_S);
    fprintf(fid, 'dt(s): %.1f\n', DT);
    fprintf(fid, 'Approach length to center (m): %.1f\n', APPROACH_LEN_M);
    fprintf(fid, 'Train per approach: %d\n', N_TRAIN_PER_APPROACH);
    fprintf(fid, 'Test trajectory: %s | approach=%s maneuver=%s\n', ...
        'Test_t', TEST_APPROACH, TEST_MANEUVER);
    fclose(fid);
end

% Write per-direction test files (for run_all_tests_t3.m)
DIR_SET = {'S', 'W', 'E'};
for d = 1:numel(DIR_SET)
    dir_d = DIR_SET{d};
    man_d = default_test_maneuver_t(dir_d);
    name_d = sprintf('Test_t_%s_to_%s', dir_d, man_d);
    file_d = fullfile(SCRIPT_DIR, sprintf('test_generated_t3_%s.csv', dir_d));
    write_single_test(file_d, name_d, dir_d, man_d, APPROACH_LEN_M, TURN_RADIUS_HINT, TIMES);
end

fprintf('Saved: %s\n', OUT_CSV);
fprintf('Summary: %s\n', OUT_SUMMARY);
fprintf('Per-direction tests: test_generated_t3_[S|W|E].csv\n');


function rows = append_rows(rows, scenario, times, xy)
    for k = 1:numel(times)
        rows(end+1, :) = {scenario, k, xy(k,1), xy(k,2)}; %#ok<AGROW>
    end
end

function maneuvers = build_maneuvers_t(approach, n_total, dist)
    if strcmpi(approach, 'S')
        n_left = round(n_total * dist.S.left);
        n_right = n_total - n_left;
        maneuvers = [repmat({'left'}, n_left, 1); repmat({'right'}, n_right, 1)];
    elseif strcmpi(approach, 'W')
        n_straight = round(n_total * dist.W.straight);
        n_right = n_total - n_straight;
        maneuvers = [repmat({'straight'}, n_straight, 1); repmat({'right'}, n_right, 1)];
    else % 'E'
        n_straight = round(n_total * dist.E.straight);
        n_left = n_total - n_straight;
        maneuvers = [repmat({'straight'}, n_straight, 1); repmat({'left'}, n_left, 1)];
    end
end

function [x, y] = generate_route_t(approach, maneuver, times, approach_len, turn_radius_hint, with_noise)
    center = [0, 0];

    switch upper(approach)
        case 'S'
            u_in = [0, 1];
            lane_offset = [1.75, 0];
        case 'W'
            u_in = [1, 0];
            lane_offset = [0, -1.75];
        otherwise % 'E'
            u_in = [-1, 0];
            lane_offset = [0, 1.75];
    end

    if strcmpi(maneuver, 'straight')
        u_out = u_in;
    elseif strcmpi(maneuver, 'left')
        u_out = [-u_in(2), u_in(1)];
    else
        u_out = [u_in(2), -u_in(1)];
    end

    p_start = center - u_in * approach_len + lane_offset;
    p_end = center + u_out * approach_len + lane_offset;
    p_in = center - u_in * turn_radius_hint + lane_offset;
    p_out = center + u_out * turn_radius_hint + lane_offset;

    if strcmpi(maneuver, 'straight')
        base = [linspace(p_start(1), p_end(1), 300)', linspace(p_start(2), p_end(2), 300)'];
    else
        seg1 = [linspace(p_start(1), p_in(1), 140)', linspace(p_start(2), p_in(2), 140)'];
        c1 = p_in + u_in * 10;
        c2 = p_out - u_out * 10;
        bez = bezier2d(p_in, c1, c2, p_out, 90);
        seg2 = [linspace(p_out(1), p_end(1), 140)', linspace(p_out(2), p_end(2), 140)'];
        base = [seg1; bez; seg2];
    end

    [x, y] = apply_behavior_profile(base, times, maneuver, with_noise);
end

function [x, y] = apply_behavior_profile(base, times, maneuver, with_noise)
    base = unique(base, 'rows', 'stable');
    s = [0; cumsum(sqrt(sum(diff(base).^2, 2)))];
    route_len = s(end);

    d_center = hypot(base(:,1), base(:,2));
    [~, i_center] = min(d_center);
    s_center = s(i_center);

    t = times(:);
    T = t(end);
    n = numel(t);
    s_lin = linspace(0, route_len, n)';

    v_nom = route_len / max(T, 1e-6);

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

    lane_shift = 1.2;
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
end

function write_single_test(file_d, name_d, dir_d, man_d, approach_len, turn_radius_hint, times)
    fid = fopen(file_d, 'w');
    if fid < 0
        error('Cannot write %s', file_d);
    end
    [x, y] = generate_route_t(dir_d, man_d, times, approach_len, turn_radius_hint, false);
    for t = 1:numel(x)
        fprintf(fid, '%s,%.3f,%.3f\n', name_d, x(t), y(t));
    end
    fprintf(fid, 'SPLIT,NaN,NaN\n');
    fclose(fid);
end

function maneuver = default_test_maneuver_t(dir_in)
    switch upper(dir_in)
        case 'S'
            maneuver = 'left';
        case 'W'
            maneuver = 'straight';
        otherwise
            maneuver = 'straight'; % E
    end
end

function pts = bezier2d(p0, p1, p2, p3, n)
    u = linspace(0, 1, n)';
    pts = ((1-u).^3) * p0 + ...
          (3*(1-u).^2 .* u) * p1 + ...
          (3*(1-u) .* u.^2) * p2 + ...
          (u.^3) * p3;
end
