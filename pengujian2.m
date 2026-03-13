clear;
clc;
close all;

% ------------------------------------------------------------
% Pengujian akurasi:
% 1) Dengan p_base (DB + behavior cues, fallback heuristic jika DB miss)
% 2) Tanpa p_base (heuristic-only dari slowdown + lateral drift)
% Output utama: bar chart dua kolom untuk paper.
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
TEST_FILE = fullfile(SCRIPT_DIR, 'test_intersection_4way3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Intersection4way3');
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');

cfg = struct();
cfg.grid_size = 10.0;
cfg.intersection_radius = 25;
cfg.window = 12;
cfg.turn_angle_thresh = 35;
cfg.min_points = 8;
cfg.alpha = 0.9;
cfg.lane_shift_ref = 1.2;
cfg.slow_gain = 1.0;
cfg.drift_gain = 0.6;

if ~exist(TEST_FILE, 'file')
    error('Test file not found: %s', TEST_FILE);
end
if ~exist(DB_FOLDER, 'dir')
    error('DB folder not found: %s', DB_FOLDER);
end
if ~exist(FIG_DIR, 'dir')
    mkdir(FIG_DIR);
end

fprintf('=== PENGUJIAN AKURASI DENGAN / TANPA p_base ===\n');
fprintf('Test file: %s\n', TEST_FILE);
fprintf('DB folder: %s\n\n', DB_FOLDER);

tracks = load_tracks_by_scenario(TEST_FILE);
db = load_database(DB_FOLDER);

res_with = evaluate_mode(tracks, db, cfg, true);
res_without = evaluate_mode(tracks, db, cfg, false);

fprintf('--- Dengan p_base ---\n');
print_result(res_with);
fprintf('\n--- Tanpa p_base (heuristic-only) ---\n');
print_result(res_without);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 760 470]);
vals = [res_with.accuracy, res_without.accuracy];
b = bar(vals, 0.55);
b.FaceColor = 'flat';
b.CData = [0.18 0.45 0.84; 0.90 0.36 0.10];

ylim([0, 100]);
ylabel('Accuracy (%)');
set(gca, 'XTick', 1:2, 'XTickLabel', {'Dengan p_{base}', 'Tanpa p_{base}'});
title('Akurasi Dengan/Tanpa p_{base} (4-Way)');
grid on;
box on;

for i = 1:2
    text(i, vals(i) + 1, sprintf('%.2f%%', vals(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

note = sprintf('N(with)=%d | N(without)=%d', res_with.total, res_without.total);
text(1.5, 6, note, 'HorizontalAlignment', 'center', 'FontSize', 9, ...
    'BackgroundColor', [1 1 1], 'Margin', 3);

png_out = fullfile(FIG_DIR, 'accuracy_with_without_pbase.png');
pdf_out = fullfile(FIG_DIR, 'accuracy_with_without_pbase.pdf');
print(fig, png_out, '-dpng', '-r220');
print(fig, pdf_out, '-dpdf', '-bestfit');
close(fig);

summary_csv = fullfile(FIG_DIR, 'accuracy_with_without_pbase_summary.csv');
fid = fopen(summary_csv, 'w');
if fid >= 0
    fprintf(fid, 'mode,accuracy,total,correct,skipped,db_used,heuristic_used\n');
    fprintf(fid, 'with_pbase,%.6f,%d,%d,%d,%d,%d\n', ...
        res_with.accuracy, res_with.total, res_with.correct, res_with.skipped, ...
        res_with.db_used, res_with.heuristic_used);
    fprintf(fid, 'without_pbase,%.6f,%d,%d,%d,%d,%d\n', ...
        res_without.accuracy, res_without.total, res_without.correct, res_without.skipped, ...
        res_without.db_used, res_without.heuristic_used);
    fclose(fid);
end

fprintf('\nFigure saved:\n- %s\n- %s\n', png_out, pdf_out);
fprintf('Summary saved:\n- %s\n', summary_csv);


function result = evaluate_mode(tracks, db, cfg, use_pbase)
    conf = zeros(3, 3);
    total = 0;
    correct = 0;
    skipped = 0;
    db_used = 0;
    heuristic_used = 0;

    for s = 1:numel(tracks)
        xy = tracks{s}.xy;
        if size(xy, 1) < cfg.min_points
            skipped = skipped + 1;
            continue;
        end

        x = xy(:, 1);
        y = xy(:, 2);
        d = hypot(x, y);
        [~, idx0] = min(d);

        i1 = max(1, idx0 - cfg.window);
        i2 = min(numel(x), idx0 + cfg.window);
        v_in = mean_vec(x, y, i1, idx0);
        v_out = mean_vec(x, y, idx0, i2);
        if any(isnan(v_in)) || any(isnan(v_out))
            skipped = skipped + 1;
            continue;
        end

        heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
        lane = lane_from_heading(heading);
        actual = turn_from_vectors(v_in, v_out, cfg.turn_angle_thresh);
        if isempty(actual)
            skipped = skipped + 1;
            continue;
        end

        pred = '';
        if use_pbase
            gx = floor(x(idx0) / cfg.grid_size) + 1;
            gy = floor(y(idx0) / cfg.grid_size) + 1;
            gid = int32((gy - 1) * 2000 + (gx - 1));

            pred = predict_turn_with_inertia( ...
                db{lane}, gid, lane, x, y, idx0, v_in, ...
                cfg.alpha, cfg.lane_shift_ref, cfg.slow_gain, cfg.drift_gain);

            if isempty(pred)
                % Fallback ke cell lain di sekitar pusat persimpangan.
                in_mask = d <= cfg.intersection_radius;
                if any(in_mask)
                    gx2 = floor(x(in_mask) / cfg.grid_size) + 1;
                    gy2 = floor(y(in_mask) / cfg.grid_size) + 1;
                    gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
                    for k = 1:numel(gids)
                        pred = predict_turn_with_inertia( ...
                            db{lane}, gids(k), lane, x, y, idx0, v_in, ...
                            cfg.alpha, cfg.lane_shift_ref, cfg.slow_gain, cfg.drift_gain);
                        if ~isempty(pred)
                            break;
                        end
                    end
                end
            end

            if ~isempty(pred)
                db_used = db_used + 1;
            end
        end

        if isempty(pred)
            pred = predict_turn_heuristic_only( ...
                x, y, idx0, v_in, cfg.lane_shift_ref, cfg.slow_gain, cfg.drift_gain);
            if ~isempty(pred)
                heuristic_used = heuristic_used + 1;
            end
        end

        if isempty(pred)
            skipped = skipped + 1;
            continue;
        end

        ai = turn_to_index(actual);
        pred_idx = turn_to_index(pred);
        conf(ai, pred_idx) = conf(ai, pred_idx) + 1;
        total = total + 1;
        if ai == pred_idx
            correct = correct + 1;
        end
    end

    result = struct();
    result.conf = conf;
    result.total = total;
    result.correct = correct;
    result.skipped = skipped;
    result.db_used = db_used;
    result.heuristic_used = heuristic_used;
    result.accuracy = 100 * correct / max(total, 1);
end

function pred = predict_turn_heuristic_only(x, y, idx0, v_in, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    if any(isnan(v_in))
        return;
    end

    [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in);
    slow_w = clamp_value((1 - speed_ratio) * slow_gain, 0, 0.6);
    drift_w = clamp_value((abs(lat_delta) / max(lane_shift_ref, 1e-6)) * drift_gain, 0, 0.6);

    p = [0.5 * slow_w, 1 - slow_w, 0.5 * slow_w];
    if lat_delta > 0
        p = [p(1) + drift_w, p(2), max(p(3) - drift_w, 0)];
    elseif lat_delta < 0
        p = [max(p(1) - drift_w, 0), p(2), p(3) + drift_w];
    end
    p = normalize_probs(p);

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

function [left_idx, straight_idx, right_idx] = lane_turn_indices(lane)
    if lane == 1
        left_idx = 7;
        straight_idx = 1;
        right_idx = 3;
    elseif lane == 2
        left_idx = 1;
        straight_idx = 3;
        right_idx = 5;
    elseif lane == 3
        left_idx = 3;
        straight_idx = 5;
        right_idx = 7;
    else
        left_idx = 5;
        straight_idx = 7;
        right_idx = 1;
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

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function v = clamp_value(v, vmin, vmax)
    v = max(vmin, min(vmax, v));
end

function print_result(r)
    fprintf('Total evaluated : %d\n', r.total);
    fprintf('Correct         : %d\n', r.correct);
    fprintf('Accuracy        : %.2f%%\n', r.accuracy);
    fprintf('Skipped         : %d\n', r.skipped);
    fprintf('DB used         : %d\n', r.db_used);
    fprintf('Heuristic used  : %d\n', r.heuristic_used);
    fprintf('Confusion matrix (row=actual, col=pred):\n');
    disp(r.conf);
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

    [uniq, ~] = unique(scenarios, 'stable');
    tracks = cell(numel(uniq), 1);
    for i = 1:numel(uniq)
        sid = uniq{i};
        m = strcmp(scenarios, sid);
        tracks{i} = struct('scenario', sid, 'xy', [xs(m), ys(m)]);
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
