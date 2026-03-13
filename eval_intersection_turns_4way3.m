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
INERTIA_ALPHA = 0.3;
LANE_SHIFT_REF_M = 1.2;

SLOW_GAIN = 2;
DRIFT_GAIN = 0.6;
ALLOW_NO_DB = true;      % allow heuristic-only prediction when DB is missing
FORCE_NO_DB = false;     % set true to ignore DB even if it exists
NO_DB_BASE = [1/3, 1/3, 1/3]; % neutral prior for fallback heuristic
ENABLE_VISUALIZATION = true; % visualize directly in this eval script
ANIM_PAUSE_SEC = 0.035;
ANIM_TAIL_LEN = 18;
ANIM_PAUSE_BETWEEN_SCENARIO_SEC = 0.20;
ANIM_PRED_DIST_M = 40;
ANIM_OVERLAP_OFFSET_M = 1.3;

% Prediction is only allowed when distance-to-center is inside this range.
PRED_START_DIST_M = 120; % start prediction when vehicle is this far (or closer) to center
PRED_END_DIST_M = 20;    % stop prediction window near center
PRED_PICK_POLICY = 'first'; % {'first','last','closest_to_end'}

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
fprintf('Prediction distance range: %.1f -> %.1f m (policy=%s)\n\n', ...
    PRED_START_DIST_M, PRED_END_DIST_M, PRED_PICK_POLICY);

tracks = load_tracks_by_scenario(TEST_FILE);
if use_db
    db = load_database(DB_FOLDER);
else
    db = cell(4, 1);
end
if ENABLE_VISUALIZATION
    viz = init_eval_visualization();
else
    viz = [];
end

labels = {'left', 'straight', 'right'};
conf = zeros(3, 3);
total = 0;
correct = 0;
skipped = 0;
db_hits = 0;
no_db_used = 0;
pred_range_miss = 0;

for s = 1:numel(tracks)
    xy = tracks{s}.xy;
    if size(xy, 1) < MIN_POINTS
        skipped = skipped + 1;
        continue;
    end

    x = xy(:, 1);
    y = xy(:, 2);
    d = hypot(x, y);
    [~, idx_center] = min(d);

    [idx_pred, pred_range_mask] = choose_prediction_index( ...
        d, idx_center, PRED_START_DIST_M, PRED_END_DIST_M, PRED_PICK_POLICY);
    if isnan(idx_pred)
        skipped = skipped + 1;
        pred_range_miss = pred_range_miss + 1;
        continue;
    end

    i1 = max(1, idx_center - PRE_POST_WINDOW);
    i2 = min(numel(x), idx_center + PRE_POST_WINDOW);

    v_in_center = mean_vec(x, y, i1, idx_center);
    v_out = mean_vec(x, y, idx_center, i2);
    if any(isnan(v_in_center)) || any(isnan(v_out))
        skipped = skipped + 1;
        continue;
    end

    ip1 = max(1, idx_pred - PRE_POST_WINDOW);
    ip2 = idx_pred;
    v_in_pred = mean_vec(x, y, ip1, ip2);
    if any(isnan(v_in_pred))
        skipped = skipped + 1;
        continue;
    end

    heading = mod(atan2(v_in_pred(1), v_in_pred(2)) * 180 / pi, 360);
    lane = lane_from_heading(heading);

    actual = turn_from_vectors(v_in_center, v_out, TURN_ANGLE_THRESH_DEG);
    if isempty(actual)
        skipped = skipped + 1;
        continue;
    end

    % Use prediction-range point as the prediction grid cell.
    gx = floor(x(idx_pred) / GRID_SIZE) + 1;
    gy = floor(y(idx_pred) / GRID_SIZE) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));

    pred = '';
    p_base = [NaN, NaN, NaN];
    p_bias = [NaN, NaN, NaN];
    p_final = [NaN, NaN, NaN];
    pred_mode = 'none';
    if use_db
        [pred, p_base, p_bias, p_final] = predict_turn_with_inertia( ...
            db{lane}, gid, lane, x, y, idx_pred, v_in_pred, ...
            INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
        if isempty(pred)
            % Fallback: search any grid inside allowed prediction range.
            cand_idx = find(pred_range_mask);
            if strcmpi(PRED_PICK_POLICY, 'last')
                cand_idx = flipud(cand_idx(:));
            end
            if ~isempty(cand_idx)
                gx2 = floor(x(cand_idx) / GRID_SIZE) + 1;
                gy2 = floor(y(cand_idx) / GRID_SIZE) + 1;
                gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
                for k = 1:numel(cand_idx)
                    [pred, p_base, p_bias, p_final] = predict_turn_with_inertia( ...
                        db{lane}, gids(k), lane, x, y, cand_idx(k), v_in_pred, ...
                        INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
                    if ~isempty(pred)
                        break;
                    end
                end
            end
        end
        if ~isempty(pred)
            db_hits = db_hits + 1;
            pred_mode = 'db+inertia';
        end
    end

    if isempty(pred) && ALLOW_NO_DB
        [pred, p_base, p_bias, p_final] = predict_turn_no_db( ...
            x, y, idx_pred, v_in_pred, NO_DB_BASE, INERTIA_ALPHA, ...
            LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
        if ~isempty(pred)
            no_db_used = no_db_used + 1;
            pred_mode = 'heuristic';
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
    if ENABLE_VISUALIZATION
        animate_eval_result( ...
            viz, x, y, idx_center, idx_pred, s, numel(tracks), tracks{s}.scenario, ...
            lane, actual, pred, p_base, p_bias, p_final, pred_mode, ...
            PRED_START_DIST_M, PRED_END_DIST_M, ANIM_PAUSE_SEC, ANIM_TAIL_LEN, ...
            ANIM_PAUSE_BETWEEN_SCENARIO_SEC, ANIM_PRED_DIST_M, ANIM_OVERLAP_OFFSET_M);
    end
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
fprintf('Skipped (outside prediction range): %d\n', pred_range_miss);
fprintf('\nConfusion matrix (rows=actual, cols=pred):\n');
fprintf('           left  straight  right\n');
for i = 1:3
    fprintf('%9s  %4d    %4d    %4d\n', labels{i}, conf(i, 1), conf(i, 2), conf(i, 3));
end

function [idx_pred, mask_pred] = choose_prediction_index(d, idx_center, pred_start_m, pred_end_m, pick_policy)
    idx_pred = NaN;
    n = numel(d);
    mask_pred = false(n, 1);
    if n < 2 || idx_center < 1 || idx_center > n
        return;
    end

    if pred_start_m < pred_end_m
        t = pred_start_m;
        pred_start_m = pred_end_m;
        pred_end_m = t;
    end

    incoming_idx = (1:idx_center)';
    d_in = d(incoming_idx);
    in_range = (d_in <= pred_start_m) & (d_in >= pred_end_m);
    mask_pred(incoming_idx(in_range)) = true;
    cand = find(mask_pred);
    if isempty(cand)
        return;
    end

    if strcmpi(pick_policy, 'last')
        idx_pred = cand(end);
    elseif strcmpi(pick_policy, 'closest_to_end')
        [~, k] = min(abs(d(cand) - pred_end_m));
        idx_pred = cand(k);
    else
        idx_pred = cand(1); % first point entering prediction range
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

function viz = init_eval_visualization()
    fig = figure('Name', 'Eval Intersection 4-Way Visualization', ...
        'NumberTitle', 'off', ...
        'Color', 'w', ...
        'Position', [120 80 1320 700]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax = nexttile(tl, 1);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');

    if exist('topology_4way3.m', 'file') == 2
        topo = topology_4way3();
        plot(topo.road_poly, 'FaceColor', [0.92 0.92 0.92], 'EdgeColor', [0.25 0.25 0.25]);
        for k = 1:numel(topo.lane_markings)
            lm = topo.lane_markings{k};
            plot(ax, lm(:, 1), lm(:, 2), '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
        end
        xlim(ax, topo.bounds(1:2));
        ylim(ax, topo.bounds(3:4));
    else
        xlim(ax, [-300 300]);
        ylim(ax, [-300 300]);
    end

    car_length = 4.4;
    car_width = 1.8;
    car_shape_local = [
        -car_width/2, -car_length/2;
         car_width/2, -car_length/2;
         car_width/2,  car_length/2;
        -car_width/2,  car_length/2
    ];

    h_center = plot(ax, 0, 0, 'p', 'MarkerFaceColor', 'y', ...
        'MarkerEdgeColor', [0.2 0.2 0.2], 'MarkerSize', 12);
    h_tail = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.8);
    h_car = patch(ax, NaN, NaN, [0.1 0.35 0.9], ...
        'FaceAlpha', 0.9, 'EdgeColor', [0 0 0], 'LineWidth', 1.0);
    h_head = quiver(ax, NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    h_idx0 = plot(ax, NaN, NaN, 'y*', 'LineWidth', 1.6, 'MarkerSize', 11);
    h_pred_pt = plot(ax, NaN, NaN, 'md', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
    h_pred = plot(ax, NaN, NaN, 'rx', 'LineWidth', 2.2, 'MarkerSize', 11);
    h_actual = plot(ax, NaN, NaN, 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    h_info = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'FontSize', 9, ...
        'BackgroundColor', [1 1 1], 'Margin', 6, 'EdgeColor', [0.6 0.6 0.6]);

    legend(ax, [h_center, h_tail, h_car, h_idx0, h_pred_pt, h_pred, h_actual], ...
        {'Center', 'Track', 'Vehicle', 'Closest point', 'Prediction point', 'Predicted', 'Actual'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal');

    ax_prob = nexttile(tl, 2);
    hold(ax_prob, 'on');
    grid(ax_prob, 'on');
    xlabel(ax_prob, 'Class');
    ylabel(ax_prob, 'Probability');
    title(ax_prob, 'p\_base / p\_bias / p\_final');
    xlim(ax_prob, [0.5 3.5]);
    ylim(ax_prob, [0 1]);
    set(ax_prob, 'XTick', 1:3, 'XTickLabel', {'left', 'straight', 'right'});
    hb = bar(ax_prob, nan(3, 3), 'grouped');
    hb(1).FaceColor = [0.2 0.45 0.95]; % p_base
    hb(2).FaceColor = [0.95 0.55 0.2]; % p_bias
    hb(3).FaceColor = [0.2 0.7 0.35];  % p_final
    legend(ax_prob, {'p\_base', 'p\_bias', 'p\_final'}, 'Location', 'northoutside', ...
        'Orientation', 'horizontal');
    h_prob_note = text(ax_prob, 0.02, 0.98, '', 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'FontSize', 9, ...
        'BackgroundColor', [1 1 1], 'Margin', 6, 'EdgeColor', [0.6 0.6 0.6]);

    viz = struct('ax', ax, ...
        'h_tail', h_tail, ...
        'h_car', h_car, ...
        'h_head', h_head, ...
        'h_idx0', h_idx0, ...
        'h_pred_pt', h_pred_pt, ...
        'h_pred', h_pred, ...
        'h_actual', h_actual, ...
        'h_info', h_info, ...
        'car_shape_local', car_shape_local, ...
        'ax_prob', ax_prob, ...
        'hb_prob', hb, ...
        'h_prob_note', h_prob_note);
end

function animate_eval_result(viz, x, y, idx_center, idx_pred, seg_idx, seg_total, scenario, lane, actual, pred, p_base, p_bias, p_final, pred_mode, pred_start_m, pred_end_m, pause_sec, tail_len, pause_between, pred_dist_m, overlap_offset_m)
    p_base_plot = p_base;
    p_bias_plot = p_bias;
    p_final_plot = p_final;
    if ~all(isfinite(p_base_plot))
        p_base_plot = [0, 0, 0];
    end
    if ~all(isfinite(p_bias_plot))
        p_bias_plot = [0, 0, 0];
    end
    if ~all(isfinite(p_final_plot))
        p_final_plot = [0, 0, 0];
    end

    pred_xy = turn_marker_xy(lane, pred, pred_dist_m);
    actual_xy = turn_marker_xy(lane, actual, pred_dist_m);
    if all(isfinite(pred_xy)) && all(isfinite(actual_xy)) && norm(pred_xy - actual_xy) < 1e-6
        pred_xy = pred_xy + [overlap_offset_m, overlap_offset_m];
    end

    set(viz.h_idx0, 'XData', x(idx_center), 'YData', y(idx_center));
    set(viz.h_pred_pt, 'XData', x(idx_pred), 'YData', y(idx_pred));
    set(viz.h_pred, 'XData', pred_xy(1), 'YData', pred_xy(2));
    set(viz.h_actual, 'XData', actual_xy(1), 'YData', actual_xy(2));
    set(viz.hb_prob(1), 'YData', p_base_plot(:)');
    set(viz.hb_prob(2), 'YData', p_bias_plot(:)');
    set(viz.hb_prob(3), 'YData', p_final_plot(:)');
    set(viz.h_prob_note, 'String', sprintf([ ...
        'mode: %s\n' ...
        'pred range = [%.1f..%.1f] m\n' ...
        'd(pred point)=%.1f m\n' ...
        'p_base = [%.2f %.2f %.2f]\n' ...
        'p_bias = [%.2f %.2f %.2f]\n' ...
        'p_final= [%.2f %.2f %.2f]'], ...
        pred_mode, ...
        pred_start_m, pred_end_m, hypot(x(idx_pred), y(idx_pred)), ...
        p_base(1), p_base(2), p_base(3), ...
        p_bias(1), p_bias(2), p_bias(3), ...
        p_final(1), p_final(2), p_final(3)));

    n = numel(x);
    for i = 1:n
        s0 = max(1, i - tail_len);
        set(viz.h_tail, 'XData', x(s0:i), 'YData', y(s0:i));

        if i < n
            dx = x(i + 1) - x(i);
            dy = y(i + 1) - y(i);
        else
            dx = x(i) - x(i - 1);
            dy = y(i) - y(i - 1);
        end

        theta = atan2(dy, dx);
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        car_world = (R * viz.car_shape_local')';
        car_world(:, 1) = car_world(:, 1) + x(i);
        car_world(:, 2) = car_world(:, 2) + y(i);

        set(viz.h_car, 'XData', car_world(:, 1), 'YData', car_world(:, 2));

        dir_norm = max(hypot(dx, dy), 1e-6);
        ux = dx / dir_norm;
        uy = dy / dir_norm;
        set(viz.h_head, 'XData', x(i), 'YData', y(i), 'UData', ux * 6, 'VData', uy * 6);
        set(viz.h_info, 'String', sprintf( ...
            ['Seg %d/%d\n%s\nlane=%d\nactual=%s | pred=%s\n' ...
             'pred range=[%.1f..%.1f] m, d(pred)=%.1f m\n' ...
             'frame=%d/%d\np_base=[%.2f %.2f %.2f]\n' ...
             'p_bias=[%.2f %.2f %.2f]\np_final=[%.2f %.2f %.2f]'], ...
            seg_idx, seg_total, scenario, lane, actual, pred, ...
            pred_start_m, pred_end_m, hypot(x(idx_pred), y(idx_pred)), i, n, ...
            p_base(1), p_base(2), p_base(3), ...
            p_bias(1), p_bias(2), p_bias(3), ...
            p_final(1), p_final(2), p_final(3)));
        title(viz.ax, sprintf('Eval Visualization | %s', scenario), 'Interpreter', 'none');
        drawnow;
        pause(pause_sec);
    end

    pause(pause_between);
end

function xy = turn_marker_xy(lane, turn_name, dist_m)
    switch lane
        case 1
            u_in = [0, 1];
        case 2
            u_in = [1, 0];
        case 3
            u_in = [0, -1];
        otherwise
            u_in = [-1, 0];
    end

    if strcmpi(turn_name, 'left')
        u_out = [-u_in(2), u_in(1)];
    elseif strcmpi(turn_name, 'right')
        u_out = [u_in(2), -u_in(1)];
    else
        u_out = u_in;
    end
    xy = dist_m * u_out;
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

function [pred, p_base, p_bias, p_final] = predict_turn_with_inertia(map, gid, lane, x, y, idx0, v_in, alpha, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    p_base = [NaN, NaN, NaN];
    p_bias = [NaN, NaN, NaN];
    p_final = [NaN, NaN, NaN];
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

function [pred, p_base, p_bias, p_final] = predict_turn_no_db(x, y, idx0, v_in, base_probs, alpha, lane_shift_ref, slow_gain, drift_gain)
    pred = '';
    p_base = [NaN, NaN, NaN];
    p_bias = [NaN, NaN, NaN];
    p_final = [NaN, NaN, NaN];
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
