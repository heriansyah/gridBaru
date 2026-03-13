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

% Probabilitas kondisi lampu merah untuk manuver belok (left/right).
P_TURN_REDLIGHT = 0.20;

% Straight behavior mix (more realistic crossing behavior).
P_STRAIGHT_NORMAL = 0.45;
P_STRAIGHT_CAUTIOUS = 0.35;
P_STRAIGHT_REDLIGHT = 0.20;
if abs((P_STRAIGHT_NORMAL + P_STRAIGHT_CAUTIOUS + P_STRAIGHT_REDLIGHT) - 1.0) > 1e-9
    error('Straight behavior probabilities must sum to 1.0');
end

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
straight_behavior_counts = struct('normal', 0, 'cautious', 0, 'redlight', 0);
turn_redlight_counts = struct('left', 0, 'right', 0);

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

        behavior_mode = 'normal';
        if strcmpi(mv, 'straight')
            behavior_mode = sample_straight_mode( ...
                P_STRAIGHT_NORMAL, P_STRAIGHT_CAUTIOUS, P_STRAIGHT_REDLIGHT);
            straight_behavior_counts.(behavior_mode) = straight_behavior_counts.(behavior_mode) + 1;
            scenario = sprintf('Train_4way_%s_Run_%03d_straight_%s', approach, i, behavior_mode);
        elseif strcmpi(mv, 'left') || strcmpi(mv, 'right')
            if rand() < P_TURN_REDLIGHT
                behavior_mode = 'redlight';
                turn_redlight_counts.(mv) = turn_redlight_counts.(mv) + 1;
                scenario = sprintf('Train_4way_%s_Run_%03d_%s_redlight', approach, i, mv);
            else
                scenario = sprintf('Train_4way_%s_Run_%03d_%s', approach, i, mv);
            end
        else
            scenario = sprintf('Train_4way_%s_Run_%03d_%s', approach, i, mv);
        end

        xy = generate_route_4way(approach, mv, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, true, behavior_mode);
        rows = append_rows(rows, scenario, TIMES, xy);
    end
end

xy_test = generate_route_4way(TEST_APPROACH, TEST_MANEUVER, TIMES, APPROACH_LEN_M, TURN_RADIUS_HINT, false, 'normal');
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
    fprintf(fid, 'Turn redlight probability (left/right): %.2f\n', P_TURN_REDLIGHT);
    fprintf(fid, 'Turn redlight counts (train only): left=%d, right=%d\n', ...
        turn_redlight_counts.left, turn_redlight_counts.right);
    fprintf(fid, 'Straight behavior distribution (train only): normal/cautious/redlight = %.2f/%.2f/%.2f\n', ...
        P_STRAIGHT_NORMAL, P_STRAIGHT_CAUTIOUS, P_STRAIGHT_REDLIGHT);
    fprintf(fid, 'Straight behavior counts (train only): normal=%d, cautious=%d, redlight=%d\n', ...
        straight_behavior_counts.normal, straight_behavior_counts.cautious, straight_behavior_counts.redlight);
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

function xy = generate_route_4way(approach, maneuver, times, approach_len, turn_radius_hint, with_noise, behavior_mode)
    if nargin < 7 || isempty(behavior_mode)
        behavior_mode = 'normal';
    end

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

    % Find closest point to center to anchor slowdown / lane shift.
    d_center = hypot(base(:,1), base(:,2));
    [~, i_center] = min(d_center);
    s_center = s(i_center);

    t = times(:);
    T = t(end);
    n = numel(t);
    s_lin = linspace(0, route_len, n)';

    v_nom = route_len / max(T, 1e-6);

    min_speed = 0.8;

    % Slowdown profile:
    % - straight: normal / cautious / redlight-stopgo
    % - turns: normal turn slowdown, or redlight-stopgo before turning
    if strcmpi(maneuver, 'straight')
        mode = lower(behavior_mode);
        if strcmp(mode, 'cautious')
            slow_profile = 1 - 0.28 * exp(-((s_lin - s_center) / 28).^2);
        elseif strcmp(mode, 'redlight')
            stop_shift = 0.0;
            if with_noise
                stop_shift = 4.0 * randn();
            end
            s_stop = s_center + stop_shift;
            gate = 0.5 * (tanh((s_lin - (s_stop - 10)) / 4) - tanh((s_lin - (s_stop + 10)) / 4));
            dip = exp(-((s_lin - s_stop) / 6).^2);
            slow_profile = (1 - 0.75 * gate) .* (1 - 0.70 * dip);
            restart_boost = 1 + 0.20 * 0.5 * (1 + tanh((s_lin - (s_stop + 12)) / 8));
            slow_profile = slow_profile .* restart_boost;
            min_speed = 0.2;
        else
            % Even "normal straight" has mild caution near crossing.
            slow_profile = 1 - 0.08 * exp(-((s_lin - s_center) / 35).^2);
        end
    else
        mode = lower(behavior_mode);
        if strcmp(mode, 'redlight')
            stop_shift = 0.0;
            if with_noise
                stop_shift = 3.0 * randn();
            end
            s_stop = s_center - 8 + stop_shift; % berhenti sedikit sebelum pusat
            gate = 0.5 * (tanh((s_lin - (s_stop - 9)) / 4) - tanh((s_lin - (s_stop + 9)) / 4));
            dip = exp(-((s_lin - s_stop) / 6).^2);
            slow_profile = (1 - 0.72 * gate) .* (1 - 0.65 * dip);
            restart_boost = 1 + 0.18 * 0.5 * (1 + tanh((s_lin - (s_stop + 10)) / 8));
            slow_profile = slow_profile .* restart_boost;
            min_speed = 0.2;
        else
            slow_amp = 0.35;
            slow_sigma = 25;
            slow_profile = 1 - slow_amp * exp(-((s_lin - s_center) / slow_sigma).^2);
        end
    end
    slow_profile = max(0.05, slow_profile);

    if with_noise
        v_nom = v_nom * (1 + 0.08 * randn());
        v_nom = max(3.0, min(v_nom, 8.0));
        phase = 2*pi*rand();
        speed = v_nom * slow_profile .* (1 + 0.08 * sin(2*pi*t/T + phase));
    else
        speed = v_nom * slow_profile;
    end

    speed = max(min_speed, speed);
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

function mode = sample_straight_mode(p_normal, p_cautious, p_redlight)
    r = rand();
    if r < p_normal
        mode = 'normal';
    elseif r < p_normal + p_cautious
        mode = 'cautious';
    else
        mode = 'redlight';
    end
    if p_redlight <= 0
        mode = 'normal';
    end
end

function pts = bezier2d(p0, p1, p2, p3, n)
    u = linspace(0, 1, n)';
    pts = ((1-u).^3) * p0 + ...
          (3*(1-u).^2 .* u) * p1 + ...
          (3*(1-u) .* u.^2) * p2 + ...
          (u.^3) * p3;
end
