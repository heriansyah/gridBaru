clear;
clc;
close all;

% ------------------------------------------------------------
% Generate figures for paper: confusion matrices and accuracy bar.
% Outputs PNG and PDF in ./figs
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');
if ~exist(FIG_DIR, 'dir')
    mkdir(FIG_DIR);
end

labels = {'left','straight','right'};

cfg4 = struct( ...
    'test_file', fullfile(SCRIPT_DIR, 'test_intersection_4way3.csv'), ...
    'db_folder', fullfile(SCRIPT_DIR, 'DB_Intersection4way3'), ...
    'grid_size', 10.0, ...
    'radius', 25, ...
    'window', 12, ...
    'angle_thresh', 35, ...
    'min_points', 8, ...
    'alpha', 0.9, ...
    'lane_shift_ref', 1.2, ...
    'slow_gain', 2.0, ...
    'drift_gain', 0.6, ...
    'use_label', false, ...
    'lane_from_heading', @lane_from_heading_4way, ...
    'lane_turn_indices', @lane_turn_indices_4way ...
);

cfgt = struct( ...
    'test_file', fullfile(SCRIPT_DIR, 'test_intersection_t3.csv'), ...
    'db_folder', fullfile(SCRIPT_DIR, 'DB_T3'), ...
    'grid_size', 10.0, ...
    'radius', 25, ...
    'window', 12, ...
    'angle_thresh', 35, ...
    'min_points', 8, ...
    'alpha', 0.45, ...
    'lane_shift_ref', 1.2, ...
    'slow_gain', 1.0, ...
    'drift_gain', 0.6, ...
    'use_label', true, ...
    'lane_from_heading', @lane_from_heading_4way, ...
    'lane_turn_indices', @lane_turn_indices_4way ...
);

cfgy = struct( ...
    'test_file', fullfile(SCRIPT_DIR, 'test_intersection_y3.csv'), ...
    'db_folder', fullfile(SCRIPT_DIR, 'DB_Y3'), ...
    'grid_size', 10.0, ...
    'radius', 28, ...
    'window', 12, ...
    'angle_thresh', 35, ...
    'min_points', 8, ...
    'alpha', 0.45, ...
    'lane_shift_ref', 1.2, ...
    'slow_gain', 2.0, ...
    'drift_gain', 0.6, ...
    'use_label', true, ...
    'lane_from_heading', @lane_from_heading_y, ...
    'lane_turn_indices', @lane_turn_indices_y ...
);

[conf4, acc4, total4, skipped4] = eval_turns(cfg4);
[conft, acct, totalt, skippedt] = eval_turns(cfgt);
[confy, accy, totaly, skippedy] = eval_turns(cfgy);

plot_confusion(conf4, labels, 'Confusion Matrix (4-Way)', fullfile(FIG_DIR, 'confusion_4way'));
plot_confusion(conft, labels, 'Confusion Matrix (T-Intersection)', fullfile(FIG_DIR, 'confusion_t'));
plot_confusion(confy, labels, 'Confusion Matrix (Y-Intersection)', fullfile(FIG_DIR, 'confusion_y'));

plot_accuracy_bar([acc4, acct, accy], {'4-Way','T','Y'}, fullfile(FIG_DIR, 'accuracy_bar'));

fprintf('4-Way: acc=%.2f total=%d skipped=%d\n', acc4, total4, skipped4);
fprintf('T:     acc=%.2f total=%d skipped=%d\n', acct, totalt, skippedt);
fprintf('Y:     acc=%.2f total=%d skipped=%d\n', accy, totaly, skippedy);


% ---------------------- helpers ----------------------
function [conf, acc, total, skipped] = eval_turns(cfg)
    tracks = load_tracks_by_scenario(cfg.test_file);
    db = load_database(cfg.db_folder);
    labels = {'left','straight','right'};

    conf = zeros(3, 3);
    total = 0;
    correct = 0;
    skipped = 0;

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
        lane = cfg.lane_from_heading(heading);

        if cfg.use_label
            actual = turn_from_label(tracks{s}.scenario);
            if isempty(actual)
                actual = turn_from_vectors(v_in, v_out, cfg.angle_thresh);
            end
        else
            actual = turn_from_vectors(v_in, v_out, cfg.angle_thresh);
        end
        if isempty(actual)
            skipped = skipped + 1;
            continue;
        end

        gx = floor(x(idx0) / cfg.grid_size) + 1;
        gy = floor(y(idx0) / cfg.grid_size) + 1;
        gid = int32((gy - 1) * 2000 + (gx - 1));

        pred = predict_turn_with_inertia( ...
            db{lane}, gid, lane, x, y, idx0, v_in, ...
            cfg.alpha, cfg.lane_shift_ref, cfg.slow_gain, cfg.drift_gain, cfg.lane_turn_indices);
        if isempty(pred)
            in_mask = d <= cfg.radius;
            if any(in_mask)
                gx2 = floor(x(in_mask) / cfg.grid_size) + 1;
                gy2 = floor(y(in_mask) / cfg.grid_size) + 1;
                gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
                for k = 1:numel(gids)
                    pred = predict_turn_with_inertia( ...
                        db{lane}, gids(k), lane, x, y, idx0, v_in, ...
                        cfg.alpha, cfg.lane_shift_ref, cfg.slow_gain, cfg.drift_gain, cfg.lane_turn_indices);
                    if ~isempty(pred)
                        break;
                    end
                end
            end
        end

        if isempty(pred)
            skipped = skipped + 1;
            continue;
        end

        ai = find(strcmp(labels, actual), 1);
        pi_idx = find(strcmp(labels, pred), 1);
        conf(ai, pi_idx) = conf(ai, pi_idx) + 1;
        total = total + 1;
        if ai == pi_idx
            correct = correct + 1;
        end
    end

    acc = 100 * correct / max(total, 1);
end

function plot_confusion(conf, labels, ttl, out_base)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 520 420]);
    imagesc(conf);
    colormap('parula');
    colorbar;
    axis equal tight;
    ax = gca;
    ax.XTick = 1:3;
    ax.YTick = 1:3;
    ax.XTickLabel = labels;
    ax.YTickLabel = labels;
    xlabel('Predicted');
    ylabel('Actual');
    title(ttl);
    for i = 1:3
        for j = 1:3
            text(j, i, num2str(conf(i, j)), 'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 10);
        end
    end
    print(fig, [out_base '.png'], '-dpng', '-r200');
    print(fig, [out_base '.pdf'], '-dpdf');
    close(fig);
end

function plot_accuracy_bar(accs, labels, out_base)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 520 360]);
    b = bar(accs);
    b.FaceColor = 'flat';
    b.CData = [0.30 0.47 0.66; 0.96 0.52 0.11; 0.33 0.64 0.35];
    ylim([0 100]);
    ylabel('Accuracy (%)');
    title('Turn Prediction Accuracy');
    ax = gca;
    ax.XTick = 1:numel(labels);
    ax.XTickLabel = labels;
    for i = 1:numel(accs)
        text(i, accs(i) + 1, sprintf('%.1f%%', accs(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 9);
    end
    print(fig, [out_base '.png'], '-dpng', '-r200');
    print(fig, [out_base '.pdf'], '-dpdf');
    close(fig);
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

function lane = lane_from_heading_4way(heading_deg)
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

function lane = lane_from_heading_y(heading_deg)
    if heading_deg >= 315 || heading_deg < 60
        lane = 1;
    elseif heading_deg >= 60 && heading_deg < 180
        lane = 3;
    else
        lane = 4;
    end
end

function [left_idx, straight_idx, right_idx] = lane_turn_indices_4way(lane)
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

function [left_idx, straight_idx, right_idx] = lane_turn_indices_y(lane)
    if lane == 1
        left_idx = 8;     % NW
        straight_idx = 9; % no north arm
        right_idx = 2;    % NE
    elseif lane == 3
        left_idx = 9;     % not used
        straight_idx = 2; % to NE
        right_idx = 5;    % to S
    else
        left_idx = 5;     % to S
        straight_idx = 8; % to NW
        right_idx = 9;    % not used
    end
end

function pred = predict_turn_with_inertia(map, gid, lane, x, y, idx0, v_in, alpha, lane_shift_ref, slow_gain, drift_gain, lane_map_fn)
    pred = '';
    if ~isKey(map, gid)
        return;
    end
    probs = map(gid);
    [left_idx, straight_idx, right_idx] = lane_map_fn(lane);
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
