% mygrid3.m - Grid-based trajectory prediction (Smart + Inertia)
% Usage:
%   mygrid3('predict', 'test_generated3.csv', node_id, window, target_distance)

function exit_code = mygrid3(mode, csv_path, varargin)
    if strcmp(mode, 'predict')
        if numel(varargin) < 3
            node_id = 0; %#ok<NASGU>
            window = 10;
            target_distance = 10;
        else
            node_id = varargin{1}; %#ok<NASGU>
            window = varargin{2};
            target_distance = varargin{3};
        end

        exit_code = run_prediction(csv_path, window, target_distance);
    else
        error('Unknown mode: %s', mode);
    end
end

function exit_code = run_prediction(csv_path, window, target_distance)
    GRID_SIZE = 10.0;
    SCRIPT_DIR = fileparts(mfilename('fullpath'));
    DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Generated3');

    if ~exist(csv_path, 'file')
        candidate = fullfile(SCRIPT_DIR, csv_path);
        if exist(candidate, 'file')
            csv_path = candidate;
        end
    end

    fprintf('=== GRID PREDICTION (PER-SEGMENT) ===\n');
    fprintf('File: %s\n', csv_path);
    fprintf('Grid: %.1fm, Target: %.1fm\n\n', GRID_SIZE, target_distance);

    if ~exist(DB_FOLDER, 'dir')
        error('Database folder not found: %s', DB_FOLDER);
    end
    if ~exist(csv_path, 'file')
        error('Test CSV not found: %s', csv_path);
    end

    tracks = load_test_tracks(csv_path);
    if isempty(tracks)
        error('No test segments found in %s', csv_path);
    end

    db = load_database(DB_FOLDER);

    fprintf('Segments found: %d\n', numel(tracks));
    fprintf('Window: %d\n\n', window);

    summary = repmat(empty_metric(), numel(tracks), 1);

    for s = 1:numel(tracks)
        m = evaluate_segment(tracks{s}.xy, db, GRID_SIZE, window);
        summary(s) = m;

        if m.total > 0
            fprintf(['Seg %3d | %-26s | pts=%4d | pred=%4d | hit=%4d | ', ...
                     'acc=%6.2f%% | mean_err=%5.3f | <=1=%6.2f%% | db=%4d | inertia=%4d\n'], ...
                    s, shrink_label(tracks{s}.scenario, 26), size(tracks{s}.xy, 1), ...
                    m.total, m.correct, 100 * m.correct / m.total, ...
                    m.mean_error, 100 * m.within1 / m.total, ...
                    m.db_hits, m.inertia_rescues);
        else
            fprintf('Seg %3d | %-26s | pts=%4d | pred=   0 | skipped (too short or invalid)\n', ...
                    s, shrink_label(tracks{s}.scenario, 26), size(tracks{s}.xy, 1));
        end
    end

    agg = aggregate_metrics(summary);

    fprintf('\n=== AGGREGATED RESULTS ===\n');
    if agg.total > 0
        fprintf('Total Predictions: %d\n', agg.total);
        fprintf('Database Hits:     %d\n', agg.db_hits);
        fprintf('Lookup Fails:      %d\n', agg.lookup_fails);
        fprintf('Inertia Rescues:   %d\n', agg.inertia_rescues);
        fprintf('Accuracy (exact):  %.2f%%\n', 100 * agg.correct / agg.total);
        fprintf('Mean Grid Error:   %.3f\n', agg.mean_error);
        fprintf('Within 1 Grid:     %.2f%%\n', 100 * agg.within1 / agg.total);
    else
        fprintf('No valid predictions across all segments.\n');
    end

    exit_code = 1;
end

function m = evaluate_segment(samples, db, grid_size, window)
    MIN_DIST_THRESHOLD = 1.5;  % in grid-step units over window
    MIN_BETA = 20.0;
    MAX_BETA = 120.0;
    ALPHA_MIN = 0.1;
    ALPHA_MAX = 0.8;

    m = empty_metric();

    if size(samples, 1) < max(window, 6) + 1
        return;
    end

    grid_seq = gps_to_grid_sequence(samples, grid_size);
    start_idx = max(window, 6);

    dir_map_matrix = [0 1; 1 1; 1 0; 1 -1; 0 -1; -1 -1; -1 0; -1 1; 0 0];

    for i = start_idx:(size(grid_seq, 1) - 1)
        cur_gx = grid_seq(i, 1);
        cur_gy = grid_seq(i, 2);

        recent = grid_seq((i - window + 1):i, 1:2);
        dx = mean(diff(recent(:, 1)));
        dy = mean(diff(recent(:, 2)));

        if dx == 0 && dy == 0
            continue;
        end

        % Compass heading: 0=N, 90=E, 180=S, 270=W
        heading_deg = mod(atan2(dx, dy) * 180 / pi, 360);
        lane = lane_from_heading(heading_deg);
        gid = int32((cur_gy - 1) * 2000 + (cur_gx - 1));

        valid_prediction = false;
        max_dir = 9;
        used_db = false;

        if isKey(db{lane}, gid)
            p_base = db{lane}(gid);
            p_base(9) = 0;
            p_base = normalize_probs(p_base);
            used_db = true;
            valid_prediction = true;
        else
            if abs(dx) > 0 || abs(dy) > 0
                p_base = zeros(1, 9);
                p_base(heading_to_direction(heading_deg)) = 1.0;
                valid_prediction = true;
            end
        end

        if ~valid_prediction
            continue;
        end

        % Hybrid Direction Boosting with Inertia Bias
        path_dist = sum(abs(diff(recent(:, 1))) + abs(diff(recent(:, 2))));
        p_final = p_base;
        if path_dist >= MIN_DIST_THRESHOLD
            feats = extract_features(recent);
            beta = predict_beta(feats, MIN_BETA, MAX_BETA);
            alpha = clamp_value(1.0 - (beta / MAX_BETA), ALPHA_MIN, ALPHA_MAX);

            heading_reg = heading_from_regression(recent);
            p_dir = directional_mass(heading_reg, beta);
            p_final = normalize_probs((1.0 - alpha) * p_base + alpha * p_dir);
        end

        [~, max_dir] = max(p_final);

        delta = dir_map_matrix(max_dir, :);
        pred_gx = cur_gx + delta(1);
        pred_gy = cur_gy + delta(2);

        actual_gx = grid_seq(i + 1, 1);
        actual_gy = grid_seq(i + 1, 2);

        jump = abs(actual_gx - cur_gx) + abs(actual_gy - cur_gy);
        if jump > 2
            continue;
        end

        m.total = m.total + 1;
        if used_db
            m.db_hits = m.db_hits + 1;
        else
            m.lookup_fails = m.lookup_fails + 1;
            m.inertia_rescues = m.inertia_rescues + 1;
        end

        distance = abs(pred_gx - actual_gx) + abs(pred_gy - actual_gy);
        m.error_sum = m.error_sum + distance;

        if distance == 0
            m.correct = m.correct + 1;
        end
        if distance <= 1
            m.within1 = m.within1 + 1;
        end
    end

    if m.total > 0
        m.mean_error = m.error_sum / m.total;
    end
end

function agg = aggregate_metrics(summary)
    agg = empty_metric();

    for i = 1:numel(summary)
        agg.total = agg.total + summary(i).total;
        agg.correct = agg.correct + summary(i).correct;
        agg.within1 = agg.within1 + summary(i).within1;
        agg.db_hits = agg.db_hits + summary(i).db_hits;
        agg.lookup_fails = agg.lookup_fails + summary(i).lookup_fails;
        agg.inertia_rescues = agg.inertia_rescues + summary(i).inertia_rescues;
        agg.error_sum = agg.error_sum + summary(i).error_sum;
    end

    if agg.total > 0
        agg.mean_error = agg.error_sum / agg.total;
    end
end

function s = shrink_label(txt, max_len)
    if numel(txt) <= max_len
        s = txt;
    else
        s = [txt(1:max_len-3), '...'];
    end
end

function m = empty_metric()
    m = struct( ...
        'total', 0, ...
        'correct', 0, ...
        'within1', 0, ...
        'db_hits', 0, ...
        'lookup_fails', 0, ...
        'inertia_rescues', 0, ...
        'error_sum', 0, ...
        'mean_error', NaN ...
    );
end

function tracks = load_test_tracks(csv_path)
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('Failed to open CSV file: %s', csv_path);
    end

    tracks = {};
    cur_xy = zeros(0, 2);
    cur_scenario = '';

    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        parts = strsplit(line, ',');
        if numel(parts) < 3
            continue;
        end

        scenario = strtrim(parts{1});
        x = str2double(parts{2});
        y = str2double(parts{3});

        if strcmpi(scenario, 'SPLIT') || isnan(x) || isnan(y)
            if size(cur_xy, 1) > 0
                tracks{end + 1} = struct('scenario', cur_scenario, 'xy', cur_xy); %#ok<AGROW>
                cur_xy = zeros(0, 2);
                cur_scenario = '';
            end
            continue;
        end

        if isempty(cur_scenario)
            cur_scenario = scenario;
        end

        cur_xy(end + 1, :) = [x, y]; %#ok<AGROW>
    end

    fclose(fid);

    if size(cur_xy, 1) > 0
        tracks{end + 1} = struct('scenario', cur_scenario, 'xy', cur_xy); %#ok<AGROW>
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

function dir = heading_to_direction(heading_deg)
    idx = round(heading_deg / 45);
    if idx == 0 || idx == 8
        dir = 1;
    elseif idx == 1
        dir = 2;
    elseif idx == 2
        dir = 3;
    elseif idx == 3
        dir = 4;
    elseif idx == 4
        dir = 5;
    elseif idx == 5
        dir = 6;
    elseif idx == 6
        dir = 7;
    elseif idx == 7
        dir = 8;
    else
        dir = 9;
    end
end

function probs = normalize_probs(probs)
    s = sum(probs);
    if s > 0
        probs = probs / s;
    end
end

function feats = extract_features(recent)
    n = size(recent, 1);
    t = (1:n)';
    x = recent(:, 1);
    y = recent(:, 2);

    [~, r2x] = fit_line(t, x);
    [~, r2y] = fit_line(t, y);
    linearity = 0.5 * (r2x + r2y);

    spread = (max(x) - min(x)) + (max(y) - min(y));

    if n >= 3
        h = zeros(n - 1, 1);
        dxy = diff(recent, 1, 1);
        for k = 1:size(dxy, 1)
            h(k) = mod(atan2(dxy(k, 1), dxy(k, 2)) * 180 / pi, 360);
        end
        hd = zeros(numel(h) - 1, 1);
        for k = 2:numel(h)
            hd(k - 1) = ang_diff_deg(h(k), h(k - 1));
        end
        heading_change = mean(abs(hd));
    else
        heading_change = 0;
    end

    feats = struct('linearity', linearity, ...
                   'spread', spread, ...
                   'heading_change', heading_change);
end

function beta = predict_beta(feats, min_beta, max_beta)
    % Practical surrogate for regression tree:
    % higher non-linearity/spread/heading-change => larger beta.
    nonlin = 1.0 - feats.linearity;
    spread_term = min(1.0, feats.spread / 12.0);
    turn_term = min(1.0, feats.heading_change / 45.0);
    u = 0.55 * nonlin + 0.25 * spread_term + 0.20 * turn_term;
    beta = min_beta + u * (max_beta - min_beta);
    beta = clamp_value(beta, min_beta, max_beta);
end

function heading_deg = heading_from_regression(recent)
    n = size(recent, 1);
    t = (1:n)';
    [sx, ~] = fit_line(t, recent(:, 1));
    [sy, ~] = fit_line(t, recent(:, 2));
    heading_deg = mod(atan2(sx, sy) * 180 / pi, 360);
end

function p_dir = directional_mass(heading_deg, beta)
    % dir indices: 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW, 9=Stay
    dir_angles = [0 45 90 135 180 225 270 315];
    half_beta = beta / 2.0;

    p_dir = zeros(1, 9);
    total_w = 0.0;

    % Inertia bias at current cell (distance 0)
    p_dir(9) = 2.0;
    total_w = total_w + 2.0;

    % Immediate neighbors in cone (distance 1)
    for d = 1:8
        if ang_diff_deg(dir_angles(d), heading_deg) <= half_beta
            p_dir(d) = 1.0;
            total_w = total_w + 1.0;
        end
    end

    if total_w > 0
        p_dir = p_dir / total_w;
    else
        p_dir(heading_to_direction(heading_deg)) = 1.0;
    end
end

function [slope, r2] = fit_line(t, v)
    if numel(v) < 2
        slope = 0;
        r2 = 0;
        return;
    end
    p = polyfit(t, v, 1);
    slope = p(1);
    yhat = polyval(p, t);
    sse = sum((v - yhat) .^ 2);
    sst = sum((v - mean(v)) .^ 2);
    if sst <= 0
        r2 = 1;
    else
        r2 = max(0, min(1, 1 - sse / sst));
    end
end

function d = ang_diff_deg(a, b)
    d = abs(mod(a - b + 180, 360) - 180);
end

function y = clamp_value(x, lo, hi)
    y = min(hi, max(lo, x));
end

function db = load_database(db_folder)
    fprintf('Loading database from %s...\n', db_folder);
    db = cell(4, 1);

    for lane = 1:4
        fname = fullfile(db_folder, sprintf('dbx_lane%d.csv', lane));
        fid = fopen(fname, 'r');
        if fid < 0
            error('Cannot open DB file: %s', fname);
        end

        m = containers.Map('KeyType', 'int32', 'ValueType', 'any');
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
            for i = 1:9
                probs(i) = str2double(parts{i + 1});
            end
            m(gid) = probs;
        end

        fclose(fid);
        db{lane} = m;
        fprintf('  lane %d: %d grid states\n', lane, m.Count);
    end
    fprintf('Database loaded.\n\n');
end

function grid_seq = gps_to_grid_sequence(samples, grid_size)
    n = size(samples, 1);
    grid_seq = zeros(n, 2);

    for i = 1:n
        gx = floor(samples(i, 1) / grid_size) + 1;
        gy = floor(samples(i, 2) / grid_size) + 1;
        grid_seq(i, :) = [gx, gy];
    end
end
