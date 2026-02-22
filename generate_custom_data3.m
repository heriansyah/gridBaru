clear;
clc;

% ------------------------------------------------------------
% SmartGrid3 synthetic generator (4-way only for phase-1)
% - Grid size: 10m
% - Approach length to center: 200m (20 grids)
% - Training maneuver distribution per approach: 50/30/20
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
OUT_CSV = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
OUT_SUMMARY = fullfile(SCRIPT_DIR, 'custom_trajectories3_summary.txt');

DURATION_S = 50;
DT = 1.0;
TIMES = (0:DT:DURATION_S)';

APPROACH_LEN_M = 200;  % 20 grids with 10m grid size
TURN_RADIUS_HINT = 20;

P_LEFT = 0.50;
P_STRAIGHT = 0.30;
P_RIGHT = 0.20;
N_TRAIN_PER_APPROACH = 10;   % total train runs = 40

TEST_APPROACH = 'south';
TEST_MANEUVER = 'left';

approaches = {'south', 'north', 'west', 'east'};

fprintf('=== GENERATE SYNTHETIC 4-WAY DATA (SMARTGRID3) ===\n');
fprintf('Duration: %d s, dt: %.1f s\n', DURATION_S, DT);
fprintf('Approach length: %.1f m (~%.1f grids)\n', APPROACH_LEN_M, APPROACH_LEN_M/10);

n_left = round(N_TRAIN_PER_APPROACH * P_LEFT);
n_straight = round(N_TRAIN_PER_APPROACH * P_STRAIGHT);
n_right = N_TRAIN_PER_APPROACH - n_left - n_straight;

rows = cell(0, 4);
run_id = 0;

for a = 1:numel(approaches)
    approach = approaches{a};
    maneuvers = [repmat({'left'}, n_left, 1); ...
                repmat({'straight'}, n_straight, 1); ...
                repmat({'right'}, n_right, 1)];

    rng(100 + a);
    maneuvers = maneuvers(randperm(numel(maneuvers)));

    for i = 1:numel(maneuvers)
        run_id = run_id + 1;
        mv = maneuvers{i};
        xy = generate_route_4way(approach, mv, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, true);
        scenario = sprintf('Train_4way_%s_Run_%03d_%s', approach, i, mv);
        rows = append_rows(rows, scenario, TIMES, xy);
    end
end

xy_test = generate_route_4way(TEST_APPROACH, TEST_MANEUVER, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, false);
rows = append_rows(rows, 'Test_4way_Veh_001_left', TIMES, xy_test);

% Write CSV
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
    fprintf(fid, 'SmartGrid3 synthetic 4-way generation summary\n');
    fprintf(fid, 'Duration(s): %d\n', DURATION_S);
    fprintf(fid, 'dt(s): %.1f\n', DT);
    fprintf(fid, 'Approach length to center (m): %.1f\n', APPROACH_LEN_M);
    fprintf(fid, 'Train per approach: %d\n', N_TRAIN_PER_APPROACH);
    fprintf(fid, 'Train left/straight/right per approach: %d/%d/%d\n', n_left, n_straight, n_right);
    fprintf(fid, 'Total train runs: %d\n', numel(approaches) * N_TRAIN_PER_APPROACH);
    fprintf(fid, 'Test trajectory: %s | approach=%s maneuver=%s\n', 'Test_4way_Veh_001_left', TEST_APPROACH, TEST_MANEUVER);
    fclose(fid);
end

fprintf('Saved: %s\n', OUT_CSV);
fprintf('Summary: %s\n', OUT_SUMMARY);
fprintf('Next: run visualize_generated_4way3.m to verify movement.\n');


function rows = append_rows(rows, scenario, times, xy)
    for k = 1:numel(times)
        rows(end+1, :) = {scenario, k, xy(k,1), xy(k,2)}; %#ok<AGROW>
    end
end

function xy = generate_route_4way(approach, maneuver, times, approach_len, turn_radius_hint, with_noise)
    center = [0, 0];

    % Direction vectors: "to center" for each approach
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

    % Start point is 200m from center in incoming direction opposite
    p_start = center - u_in * approach_len + lane_offset;

    % Outgoing direction by maneuver
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
        base = [linspace(p_start(1), p_end(1), 320)', linspace(p_start(2), p_end(2), 320)'];
    else
        seg1 = [linspace(p_start(1), p_in(1), 140)', linspace(p_start(2), p_in(2), 140)'];
        c1 = p_in + u_in * 10;
        c2 = p_out - u_out * 10;
        bez = bezier2d(p_in, c1, c2, p_out, 90);
        seg2 = [linspace(p_out(1), p_end(1), 140)', linspace(p_out(2), p_end(2), 140)'];
        base = [seg1; bez; seg2];
    end

    base = unique(base, 'rows', 'stable');
    s = [0; cumsum(sqrt(sum(diff(base).^2, 2)))];
    route_len = s(end);

    t = times(:);
    T = t(end);

    v_nom = route_len / max(T, 1e-6);
    if with_noise
        v_nom = v_nom * (1 + 0.10 * randn());
        v_nom = max(3.0, min(v_nom, 8.0));
        phase = 2*pi*rand();
        speed = v_nom * (1 + 0.12 * sin(2*pi*t/T + phase));
    else
        speed = v_nom * ones(size(t));
    end

    speed = max(1.0, speed);
    ds = speed * mean(diff(t));
    s_target = cumsum([0; ds(1:end-1)]);

    if s_target(end) > 0
        s_target = s_target * (route_len / s_target(end));
    end

    x = interp1(s, base(:,1), s_target, 'linear', 'extrap');
    y = interp1(s, base(:,2), s_target, 'linear', 'extrap');

    if with_noise
        gx = gradient(x);
        gy = gradient(y);
        gnorm = sqrt(gx.^2 + gy.^2) + 1e-9;
        nx = -gy ./ gnorm;
        ny = gx ./ gnorm;
        jitter = 0.20 * randn(size(x));
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
