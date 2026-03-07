clear;
clc;

% ------------------------------------------------------------
% Evaluate turn accuracy at Y-intersection only (left/straight/right).
% Uses Y-specific lane mapping to match DB_Y3 layout.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
TEST_FILE = fullfile(SCRIPT_DIR, 'test_intersection_y3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Y3');

GRID_SIZE = 10.0;
INTERSECTION_RADIUS_M = 28;
PRE_POST_WINDOW = 12;
TURN_ANGLE_THRESH_DEG = 35;
MIN_POINTS = 8;
PRED_DIST_M = 40;
INERTIA_ALPHA = 0.45;
LANE_SHIFT_REF_M = 1.2;
SLOW_GAIN = 2.0;
DRIFT_GAIN = 0.6;
SHOW_VIS = true;
VIS_MODE = 'first_miss'; % 'first_miss' or 'first'

if ~exist(TEST_FILE, 'file')
    alt = fullfile(SCRIPT_DIR, 'test_generated_y3.csv');
    if exist(alt, 'file')
        TEST_FILE = alt;
    else
        alt = fullfile(SCRIPT_DIR, 'custom_trajectories_y3.csv');
        if exist(alt, 'file')
            TEST_FILE = alt;
        else
            error('Test file not found: %s', TEST_FILE);
        end
    end
end
if ~exist(DB_FOLDER, 'dir')
    error('DB folder not found: %s', DB_FOLDER);
end

fprintf('=== EVAL INTERSECTION TURN ACCURACY (Y) ===\n');
fprintf('Test file: %s\n', TEST_FILE);
fprintf('DB folder: %s\n', DB_FOLDER);
fprintf('Radius: %.1f m, Window: %d, Angle thresh: %.1f deg\n\n', ...
    INTERSECTION_RADIUS_M, PRE_POST_WINDOW, TURN_ANGLE_THRESH_DEG);

tracks = load_tracks_by_scenario(TEST_FILE);
db = load_database(DB_FOLDER);

labels = {'left', 'straight', 'right'};
conf = zeros(3, 3);
total = 0;
correct = 0;
skipped = 0;
results = repmat(struct('valid', false, 'pred', '', 'actual', '', 'lane', NaN, ...
    'heading', NaN, 'gid', NaN, 'pbase', [], 'pfinal', [], 'idx0', NaN), numel(tracks), 1);

for s = 1:numel(tracks)
    xy = tracks{s}.xy;
    if size(xy, 1) < MIN_POINTS
        skipped = skipped + 1;
        continue;
    end

    x = xy(:, 1);
    y = xy(:, 2);
    d = hypot(x, y);
    [~, idx0] = min(d);

    i1 = max(1, idx0 - PRE_POST_WINDOW);
    i2 = min(numel(x), idx0 + PRE_POST_WINDOW);

    v_in = mean_vec(x, y, i1, idx0);
    v_out = mean_vec(x, y, idx0, i2);
    if any(isnan(v_in)) || any(isnan(v_out))
        skipped = skipped + 1;
        continue;
    end

    heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
    lane = lane_from_heading_y(heading);

    actual = turn_from_label(tracks{s}.scenario);
    if isempty(actual)
        actual = turn_from_vectors(v_in, v_out, TURN_ANGLE_THRESH_DEG);
    end
    if isempty(actual)
        skipped = skipped + 1;
        continue;
    end

    gx = floor(x(idx0) / GRID_SIZE) + 1;
    gy = floor(y(idx0) / GRID_SIZE) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));

    [pred, p_base, p_final] = predict_turn_with_inertia_y( ...
        db{lane}, gid, lane, x, y, idx0, v_in, ...
        INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
    if isempty(pred)
        % Fallback: search any point within radius that exists in DB.
        in_mask = d <= INTERSECTION_RADIUS_M;
        if any(in_mask)
            gx2 = floor(x(in_mask) / GRID_SIZE) + 1;
            gy2 = floor(y(in_mask) / GRID_SIZE) + 1;
            gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
            pred = '';
            for k = 1:numel(gids)
                [pred, p_base, p_final] = predict_turn_with_inertia_y( ...
                    db{lane}, gids(k), lane, x, y, idx0, v_in, ...
                    INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
                if ~isempty(pred)
                    gid = gids(k);
                    break;
                end
            end
        end
    end

    if isempty(pred)
        skipped = skipped + 1;
        continue;
    end

    results(s).valid = true;
    results(s).pred = pred;
    results(s).actual = actual;
    results(s).lane = lane;
    results(s).heading = heading;
    results(s).gid = gid;
    results(s).pbase = p_base;
    results(s).pfinal = p_final;
    results(s).idx0 = idx0;

    ai = turn_to_index(actual);
    pi = turn_to_index(pred);
    conf(ai, pi) = conf(ai, pi) + 1;
    total = total + 1;
    if ai == pi
        correct = correct + 1;
    end

    fprintf('Seg %2d | %-24s | lane=%d | actual=%s | pred=%s\n', ...
        s, tracks{s}.scenario, lane, actual, pred);
end

fprintf('\n=== SUMMARY ===\n');
fprintf('Total evaluated: %d\n', total);
fprintf('Correct: %d\n', correct);
fprintf('Accuracy: %.2f%%\n', 100 * correct / max(total, 1));
fprintf('Skipped: %d\n', skipped);
fprintf('\nConfusion matrix (rows=actual, cols=pred):\n');
fprintf('           left  straight  right\n');
for i = 1:3
    fprintf('%9s  %4d    %4d    %4d\n', labels{i}, conf(i, 1), conf(i, 2), conf(i, 3));
end

if SHOW_VIS && total > 0
    vis_idx = pick_visual_index(results, VIS_MODE);
    if ~isnan(vis_idx)
        visualize_one(tracks{vis_idx}, results(vis_idx), PRED_DIST_M);
    end
end


function v = mean_vec(x, y, i1, i2)
    if i2 <= i1
        v = [NaN, NaN];
        return;
    end
    dx = diff(x(i1:i2));
    dy = diff(y(i1:i2));
    if isempty(dx)
        v = [NaN, NaN];
        return;
    end
    v = [mean(dx), mean(dy)];
end

function t = turn_from_label(sid)
    s = lower(sid);
    if contains(s, 'left')
        t = 'left';
    elseif contains(s, 'right')
        t = 'right';
    elseif contains(s, 'straight')
        t = 'straight';
    else
        t = '';
    end
end

function t = turn_from_vectors(v_in, v_out, thresh_deg)
    n1 = norm(v_in);
    n2 = norm(v_out);
    if n1 < 1e-6 || n2 < 1e-6
        t = '';
        return;
    end
    v1 = v_in / n1;
    v2 = v_out / n2;
    dotv = max(-1, min(1, v1(1) * v2(1) + v1(2) * v2(2)));
    ang = atan2(v1(1) * v2(2) - v1(2) * v2(1), dotv) * 180 / pi;
    if abs(ang) <= thresh_deg
        t = 'straight';
    elseif ang > 0
        t = 'left';
    else
        t = 'right';
    end
end

function idx = turn_to_index(turn)
    switch lower(turn)
        case 'left'
            idx = 1;
        case 'straight'
            idx = 2;
        otherwise
            idx = 3;
    end
end

function lane = lane_from_heading_y(heading_deg)
    % Y inbound lanes:
    % lane1: from S (northbound)
    % lane3: from NW (heading ~135)
    % lane4: from NE (heading ~225)
    if heading_deg >= 315 || heading_deg < 60
        lane = 1;  % northbound
    elseif heading_deg >= 60 && heading_deg < 180
        lane = 3;  % SE inbound
    else
        lane = 4;  % SW inbound
    end
end

function [pred, p_base, p_final] = predict_turn_with_inertia_y(map, gid, lane, x, y, idx0, v_in, alpha, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    p_base = [];
    p_final = [];
    if ~isKey(map, gid)
        return;
    end
    probs = map(gid);
    [left_idx, straight_idx, right_idx] = lane_turn_indices_y(lane);
    p_base = [probs(left_idx), probs(straight_idx), probs(right_idx)];

    [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in);
    slow_w = clamp_value((1 - speed_ratio) * slow_gain, 0, 0.6);
    drift_w = clamp_value((abs(lat_delta) / max(lane_shift_ref, 1e-6)) * drift_gain, 0, 0.6);

    p_bias = [0.5 * slow_w, 1 - slow_w, 0.5 * slow_w];
    if lat_delta > 0
        p_bias = [p_bias(1) + drift_w, p_bias(2), max(p_bias(3) - drift_w, 0)];
    elseif lat_delta < 0
        p_bias = [max(p_bias(1) - drift_w, 0), p_bias(2), p_bias(3) + drift_w];
    end
    p_bias = normalize_probs(p_bias);

    p_final = normalize_probs((1 - alpha) * p_base + alpha * p_bias);
    [~, imax] = max(p_final);
    if imax == 1
        pred = 'left';
    elseif imax == 2
        pred = 'straight';
    else
        pred = 'right';
    end
end

function idx = pick_visual_index(results, mode)
    idx = NaN;
    if strcmpi(mode, 'first_miss')
        for i = 1:numel(results)
            if results(i).valid && ~strcmp(results(i).pred, results(i).actual)
                idx = i;
                return;
            end
        end
    end
    for i = 1:numel(results)
        if results(i).valid
            idx = i;
            return;
        end
    end
end

function visualize_one(track, res, pred_dist)
    topo = topology_y3();
    x = track.xy(:, 1);
    y = track.xy(:, 2);

    fig = figure('Name', 'Eval Visualization (Y)', ...
        'NumberTitle', 'off', 'Color', 'w', 'Position', [140 90 1000 640]);
    ax = axes(fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    plot(topo.road_poly, 'FaceColor', [0.92 0.92 0.92], 'EdgeColor', [0.25 0.25 0.25]);
    for k = 1:numel(topo.lane_markings)
        lm = topo.lane_markings{k};
        plot(ax, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
    end

    h_traj = plot(ax, x, y, 'b-', 'LineWidth', 1.6);

    if ~isnan(res.idx0)
        s = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];
        s0 = s(res.idx0);
        [~, idx_target] = min(abs(s - (s0 + pred_dist)));
        actual_xy = [x(idx_target), y(idx_target)];
    else
        actual_xy = [NaN, NaN];
    end

    pred_xy = [NaN, NaN];
    if ~isempty(res.pred) && ~isnan(res.idx0)
        u_out = turn_to_vector_y(res.lane, res.pred);
        pred_xy = [x(res.idx0), y(res.idx0)] + u_out * pred_dist;
    end

    h_pred = plot(ax, pred_xy(1), pred_xy(2), 'rx', 'LineWidth', 2.2, 'MarkerSize', 11);
    h_act = plot(ax, actual_xy(1), actual_xy(2), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    lgd = legend(ax, [h_traj, h_pred, h_act], {'Trajectory', 'Predicted', 'Actual'}, 'Location', 'best');
    set(lgd, 'AutoUpdate', 'off');

    xlim(ax, topo.bounds(1:2));
    ylim(ax, topo.bounds(3:4));
    title(ax, sprintf('%s | actual=%s | pred=%s', track.scenario, res.actual, res.pred), 'Interpreter', 'none');

    info = sprintf('lane=%d | heading=%.1f° | gid=%d\npBase: L=%.2f S=%.2f R=%.2f\npFinal: L=%.2f S=%.2f R=%.2f', ...
        res.lane, res.heading, res.gid, res.pbase(1), res.pbase(2), res.pbase(3), ...
        res.pfinal(1), res.pfinal(2), res.pfinal(3));
    text(ax, 0.02, 0.98, info, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'BackgroundColor', [1 1 1], 'Margin', 6, 'EdgeColor', [0.6 0.6 0.6]);
end

function u = turn_to_vector_y(lane, turn)
    if lane == 1
        if strcmp(turn, 'left')
            u = [-1, 1] / sqrt(2);
        elseif strcmp(turn, 'right')
            u = [1, 1] / sqrt(2);
        else
            u = [0, 1];
        end
    elseif lane == 3
        if strcmp(turn, 'right')
            u = [0, -1];
        else
            u = [1, 1] / sqrt(2);
        end
    else
        if strcmp(turn, 'left')
            u = [0, -1];
        else
            u = [-1, 1] / sqrt(2);
        end
    end
end

function [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in)
    n = numel(x);
    if n < 6 || idx0 < 3 || idx0 > n - 2
        speed_ratio = 1.0;
        lat_delta = 0.0;
        return;
    end

    i_before1 = max(1, idx0 - 8);
    i_before2 = max(2, idx0 - 1);
    i_center1 = max(1, idx0 - 2);
    i_center2 = min(n, idx0 + 2);
    sp_before = mean(hypot(diff(x(i_before1:i_before2)), diff(y(i_before1:i_before2))));
    sp_center = mean(hypot(diff(x(i_center1:i_center2)), diff(y(i_center1:i_center2))));
    speed_ratio = sp_center / max(sp_before, 1e-6);

    if any(isnan(v_in))
        lat_delta = 0.0;
        return;
    end
    vin_norm = norm(v_in);
    if vin_norm < 1e-6
        lat_delta = 0.0;
        return;
    end
    u = v_in / vin_norm;
    left_n = [-u(2), u(1)];

    c = [x(idx0), y(idx0)];
    lat = (x - c(1)) * left_n(1) + (y - c(2)) * left_n(2);

    i_left1 = max(1, idx0 - 8);
    i_left2 = max(1, idx0 - 1);
    i_right1 = min(n, idx0 + 1);
    i_right2 = min(n, idx0 + 8);
    lat_before = mean(lat(i_left1:i_left2));
    lat_after = mean(lat(i_right1:i_right2));
    lat_delta = lat_after - lat_before;
end

function [left_idx, straight_idx, right_idx] = lane_turn_indices_y(lane)
    % Direction index:
    % 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW, 9=Stay
    if lane == 1
        left_idx = 8;     % NW
        straight_idx = 9; % no north arm
        right_idx = 2;    % NE
    elseif lane == 3
        left_idx = 9;     % not used
        straight_idx = 2; % to NE
        right_idx = 5;    % to S
    else % lane 4
        left_idx = 5;     % to S
        straight_idx = 8; % to NW
        right_idx = 9;    % not used
    end
end

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function v = clamp_value(v, vmin, vmax)
    v = max(vmin, min(vmax, v));
end

function tracks = load_tracks_by_scenario(fname)
    fid = fopen(fname, 'r');
    if fid < 0
        error('Could not open %s', fname);
    end

    header = fgetl(fid); %#ok<NASGU>
    data = textscan(fid, '%s %f %f %f', 'Delimiter', ',', 'CollectOutput', false);
    fclose(fid);

    scenarios = data{1};
    xs = data{3};
    ys = data{4};

    % Support 3-column files: scenario,x,y (no timestamp).
    if all(isnan(ys)) && ~all(isnan(data{2})) && ~all(isnan(data{3}))
        xs = data{2};
        ys = data{3};
    end

    valid = ~isnan(xs) & ~isnan(ys);
    scenarios = scenarios(valid);
    xs = xs(valid);
    ys = ys(valid);

    if isempty(scenarios)
        tracks = {};
        return;
    end

    [uniq, ~] = unique(scenarios, 'stable');
    tracks = cell(numel(uniq), 1);
    for i = 1:numel(uniq)
        sid = uniq{i};
        mask = strcmp(scenarios, sid);
        tracks{i} = struct('scenario', sid, 'xy', [xs(mask), ys(mask)]);
    end
end

function db = load_database(folder)
    db = cell(4, 1);
    for lane = 1:4
        fname = fullfile(folder, sprintf('dbx_lane%d.csv', lane));
        m = containers.Map('KeyType', 'int32', 'ValueType', 'any');

        if exist(fname, 'file') ~= 2
            db{lane} = m;
            continue;
        end

        fid = fopen(fname, 'r');
        if fid < 0
            db{lane} = m;
            continue;
        end

        while true
            line = fgetl(fid);
            if ~ischar(line)
                break;
            end
            parts = strsplit(line, ',');
            if numel(parts) < 10
                continue;
            end

            gid = int32(str2double(parts{1}));
            probs = zeros(1, 9);
            for k = 1:9
                probs(k) = str2double(parts{k + 1});
            end
            m(gid) = probs;
        end

        fclose(fid);
        db{lane} = m;
    end
end
