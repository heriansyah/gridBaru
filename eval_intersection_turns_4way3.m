clear;
clc;

% ------------------------------------------------------------
% Evaluate turn accuracy at intersection only (left/straight/right).
% This ignores exact coordinate accuracy.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
TEST_FILE = fullfile(SCRIPT_DIR, 'test_intersection_4way3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Intersection4way3');

GRID_SIZE = 10.0;
INTERSECTION_RADIUS_M = 25;
PRE_POST_WINDOW = 12;
TURN_ANGLE_THRESH_DEG = 35;
MIN_POINTS = 8;
INERTIA_ALPHA = 0.5;
LANE_SHIFT_REF_M = 1.2;

SLOW_GAIN = 2;
DRIFT_GAIN = 0.6;
ALLOW_NO_DB = true;      % allow heuristic-only prediction when DB is missing
FORCE_NO_DB = true;     % set true to ignore DB even if it exists
NO_DB_BASE = [0.25, 0.5, 0.25]; % left/straight/right prior (no training)

if ~exist(TEST_FILE, 'file')
    error('Test file not found: %s', TEST_FILE);
end
use_db = exist(DB_FOLDER, 'dir') == 7 && ~FORCE_NO_DB;
if ~use_db && ~ALLOW_NO_DB
    error('DB folder not found: %s', DB_FOLDER);
end

fprintf('=== EVAL INTERSECTION TURN ACCURACY (4-WAY) ===\n');
fprintf('Test file: %s\n', TEST_FILE);
if use_db
    fprintf('DB folder: %s\n', DB_FOLDER);
else
    fprintf('DB folder: (not used) -> heuristic-only mode\n');
end
fprintf('Radius: %.1f m, Window: %d, Angle thresh: %.1f deg\n\n', ...
    INTERSECTION_RADIUS_M, PRE_POST_WINDOW, TURN_ANGLE_THRESH_DEG);

tracks = load_tracks_by_scenario(TEST_FILE);
if use_db
    db = load_database(DB_FOLDER);
else
    db = cell(4, 1);
end

labels = {'left', 'straight', 'right'};
conf = zeros(3, 3);
total = 0;
correct = 0;
skipped = 0;
db_hits = 0;
no_db_used = 0;

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
    lane = lane_from_heading(heading);

    actual = turn_from_vectors(v_in, v_out, TURN_ANGLE_THRESH_DEG);
    if isempty(actual)
        skipped = skipped + 1;
        continue;
    end

    % Use closest-to-center point as intersection grid cell.
    gx = floor(x(idx0) / GRID_SIZE) + 1;
    gy = floor(y(idx0) / GRID_SIZE) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));

    pred = '';
    if use_db
        pred = predict_turn_with_inertia( ...
            db{lane}, gid, lane, x, y, idx0, v_in, ...
            INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
        if isempty(pred)
            % Fallback: search any point within radius that exists in DB.
            in_mask = d <= INTERSECTION_RADIUS_M;
            if any(in_mask)
                gx2 = floor(x(in_mask) / GRID_SIZE) + 1;
                gy2 = floor(y(in_mask) / GRID_SIZE) + 1;
                gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
                for k = 1:numel(gids)
                    pred = predict_turn_with_inertia( ...
                        db{lane}, gids(k), lane, x, y, idx0, v_in, ...
                        INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
                    if ~isempty(pred)
                        break;
                    end
                end
            end
        end
        if ~isempty(pred)
            db_hits = db_hits + 1;
        end
    end

    if isempty(pred) && ALLOW_NO_DB
        pred = predict_turn_no_db( ...
            x, y, idx0, v_in, NO_DB_BASE, INERTIA_ALPHA, ...
            LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
        if ~isempty(pred)
            no_db_used = no_db_used + 1;
        end
    end

    if isempty(pred)
        skipped = skipped + 1;
        continue;
    end

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
if ALLOW_NO_DB
    fprintf('DB hits: %d\n', db_hits);
    fprintf('Heuristic-only: %d\n', no_db_used);
end
fprintf('\nConfusion matrix (rows=actual, cols=pred):\n');
fprintf('           left  straight  right\n');
for i = 1:3
    fprintf('%9s  %4d    %4d    %4d\n', labels{i}, conf(i, 1), conf(i, 2), conf(i, 3));
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

function lane = lane_from_heading(heading_deg)
    if heading_deg >= 315 || heading_deg < 45
        lane = 1;
    elseif heading_deg >= 45 && heading_deg < 135
        lane = 2;
    elseif heading_deg >= 135 && heading_deg < 225
        lane = 3;
    else
        lane = 4;
    end
end

function pred = predict_turn_from_db(map, gid, lane)
    pred = '';
    if ~isKey(map, gid)
        return;
    end
    probs = map(gid);
    [left_idx, straight_idx, right_idx] = lane_turn_indices(lane);
    p = [probs(left_idx), probs(straight_idx), probs(right_idx)];
    [~, imax] = max(p);
    if imax == 1
        pred = 'left';
    elseif imax == 2
        pred = 'straight';
    else
        pred = 'right';
    end
end

function pred = predict_turn_with_inertia(map, gid, lane, x, y, idx0, v_in, alpha, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    if ~isKey(map, gid)
        return;
    end
    probs = map(gid);
    [left_idx, straight_idx, right_idx] = lane_turn_indices(lane);
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

function pred = predict_turn_no_db(x, y, idx0, v_in, base_probs, alpha, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    if any(isnan(v_in))
        return;
    end
    if numel(base_probs) ~= 3
        base_probs = [1, 1, 1];
    end
    p_base = normalize_probs(base_probs);

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

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function v = clamp_value(v, vmin, vmax)
    v = max(vmin, min(vmax, v));
end

function [left_idx, straight_idx, right_idx] = lane_turn_indices(lane)
    % Direction index:
    % 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW, 9=Stay
    if lane == 1
        left_idx = 7;      % W
        straight_idx = 1;  % N
        right_idx = 3;     % E
    elseif lane == 2
        left_idx = 1;      % N
        straight_idx = 3;  % E
        right_idx = 5;     % S
    elseif lane == 3
        left_idx = 3;      % E
        straight_idx = 5;  % S
        right_idx = 7;     % W
    else
        left_idx = 5;      % S
        straight_idx = 7;  % W
        right_idx = 1;     % N
    end
end

function tracks = load_tracks_by_scenario(fname)
    fid = fopen(fname, 'r');
    if fid < 0
        error('Could not open test file: %s', fname);
    end
    header = fgetl(fid); %#ok<NASGU>
    data = textscan(fid, '%s %f %f %f', 'Delimiter', ',', 'CollectOutput', false);
    fclose(fid);

    scenarios = data{1};
    xs = data{3};
    ys = data{4};

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
