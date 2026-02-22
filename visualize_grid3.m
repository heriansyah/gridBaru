clearvars -except TEST_FILE;
clc;
close all;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Generated3');
if ~exist('TEST_FILE', 'var')
    TEST_FILE = fullfile(SCRIPT_DIR, 'test_generated3.csv');
end
TOPOLOGY_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');

GRID_SIZE = 10.0;
WINDOW = 10;
STEP_STRIDE = 1;
PAUSE_SEC = 0.08;
SEGMENT_MODE = 'longest';   % 'longest' or 'index'
SEGMENT_INDEX = 1;          % used when SEGMENT_MODE='index'
SHOW_TOPOLOGY = true;       % show global road topology in background
AUTO_ZOOM = false;          % false: context zoom, true: follow vehicle
CONTEXT_RADIUS_M = 300;     % topology/context radius around selected segment (meters)
GRID_MESH_STEP = 5;         % show grid mesh every N cells on right panel
GRID_MARGIN_CELLS = 2;      % extra margin around computed grid limits

% Moving-car visual parameters (meters)
CAR_LENGTH_M = 4.4;
CAR_WIDTH_M = 1.8;

fprintf('=== VISUALIZE GRID PREDICTION ===\n');
fprintf('Test file: %s\n', TEST_FILE);
fprintf('DB folder: %s\n', DB_FOLDER);
if SHOW_TOPOLOGY
    fprintf('Topology file: %s\n', TOPOLOGY_FILE);
end
fprintf('\n');

tracks = load_test_tracks(TEST_FILE);
if isempty(tracks)
    error('No valid test segments found in %s', TEST_FILE);
end

seg_lens = cellfun(@(t) size(t.xy, 1), tracks);
fprintf('Available segments:\n');
for k = 1:numel(tracks)
    fprintf('  %3d) %-30s points=%d\n', k, tracks{k}.scenario, seg_lens(k));
end

if strcmpi(SEGMENT_MODE, 'longest')
    [~, idx] = max(seg_lens);
else
    idx = max(1, min(SEGMENT_INDEX, numel(tracks)));
end

track = tracks{idx};
samples = track.xy;
grid_seq = floor(samples ./ GRID_SIZE) + 1;

fprintf('Using segment %d/%d: scenario=%s, points=%d\n\n', ...
    idx, numel(tracks), track.scenario, size(grid_seq, 1));

db = load_database(DB_FOLDER);

if size(grid_seq, 1) < max(WINDOW, 6) + 1
    error('Selected segment has too few points for visualization (need >= %d).', max(WINDOW, 6) + 1);
end

topology_xy = zeros(0, 2);
if SHOW_TOPOLOGY && exist(TOPOLOGY_FILE, 'file') == 2
    topology_xy = load_topology_points(TOPOLOGY_FILE);
elseif SHOW_TOPOLOGY
    fprintf('Warning: topology file not found, background topology disabled.\n');
end

% Prefer explicit road geometry (solid) when available.
topo_model = [];
if SHOW_TOPOLOGY && exist(fullfile(SCRIPT_DIR, 'topology_4way3.m'), 'file') == 2
    try
        topo_model = topology_4way3();
    catch
        topo_model = [];
    end
end

fig = figure('Name', 'SmartGrid3 Dual Visualization (Segment + Topology)', ...
             'NumberTitle', 'off', ...
             'Color', 'w', ...
             'Position', [80 80 1450 820]);

tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Left panel: raw meter space
ax_raw = nexttile(tl, 1);
hold(ax_raw, 'on');
grid(ax_raw, 'on');
axis(ax_raw, 'equal');
xlabel(ax_raw, 'X (m)');
ylabel(ax_raw, 'Y (m)');
title(ax_raw, 'Raw Meter Space');

if ~isempty(topo_model)
    plot(topo_model.road_poly, 'FaceColor', [0.88 0.88 0.88], ...
        'EdgeColor', [0.35 0.35 0.35], 'LineWidth', 1.0);
    for mk = 1:numel(topo_model.lane_markings)
        lm = topo_model.lane_markings{mk};
        plot(ax_raw, lm(:,1), lm(:,2), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0);
    end
elseif ~isempty(topology_xy)
    plot(ax_raw, topology_xy(:,1), topology_xy(:,2), '.', ...
        'Color', [0.78 0.78 0.78], 'MarkerSize', 3);
end

plot(ax_raw, samples(:,1), samples(:,2), '-', ...
    'Color', [0.25 0.45 0.95], 'LineWidth', 1.2);

h_raw_tail = plot(ax_raw, NaN, NaN, 'b-', 'LineWidth', 1.9);
h_raw_pred = plot(ax_raw, NaN, NaN, 'rx', 'LineWidth', 2.2, 'MarkerSize', 11);
h_raw_actual = plot(ax_raw, NaN, NaN, 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

car_shape_local = [
    -CAR_WIDTH_M/2, -CAR_LENGTH_M/2;
     CAR_WIDTH_M/2, -CAR_LENGTH_M/2;
     CAR_WIDTH_M/2,  CAR_LENGTH_M/2;
    -CAR_WIDTH_M/2,  CAR_LENGTH_M/2
];

h_raw_car = patch(ax_raw, NaN, NaN, [0.1 0.35 0.9], ...
    'FaceAlpha', 0.9, 'EdgeColor', [0 0 0], 'LineWidth', 1.0);
h_raw_head = quiver(ax_raw, NaN, NaN, NaN, NaN, 0, ...
    'r', 'LineWidth', 1.8, 'MaxHeadSize', 2);

% Right panel: grid space
ax_grid = nexttile(tl, 2);
hold(ax_grid, 'on');
grid(ax_grid, 'off');
axis(ax_grid, 'equal');
xlabel(ax_grid, 'Grid X');
ylabel(ax_grid, 'Grid Y');
title(ax_grid, 'Grid Space');

if ~isempty(topo_model)
    draw_topology_grid(ax_grid, topo_model, GRID_SIZE);
elseif ~isempty(topology_xy)
    topo_grid = floor(topology_xy ./ GRID_SIZE) + 1;
    plot(ax_grid, topo_grid(:,1), topo_grid(:,2), '.', ...
        'Color', [0.78 0.78 0.78], 'MarkerSize', 3);
else
    topo_grid = zeros(0,2);
end

plot(ax_grid, grid_seq(:,1), grid_seq(:,2), '-', ...
    'Color', [0.25 0.45 0.95], 'LineWidth', 1.2);

h_grid_tail = plot(ax_grid, NaN, NaN, 'b-', 'LineWidth', 1.9);
h_grid_curr = plot(ax_grid, NaN, NaN, 'o', 'Color', [0.1 0.4 1.0], ...
    'MarkerFaceColor', [0.1 0.4 1.0], 'MarkerEdgeColor', 'w', 'LineWidth', 1.0, 'MarkerSize', 10);
h_grid_pred = plot(ax_grid, NaN, NaN, 'x', 'Color', [1.0 0.1 0.1], 'LineWidth', 2.6, 'MarkerSize', 12);
h_grid_actual = plot(ax_grid, NaN, NaN, 's', 'Color', [0.2 0.9 0.2], ...
    'MarkerFaceColor', [0.2 0.9 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'MarkerSize', 9);
h_grid_link_pred = plot(ax_grid, NaN, NaN, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
h_grid_link_actual = plot(ax_grid, NaN, NaN, 'g--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
h_grid_text = text(ax_grid, 0, 0, '', ...
    'Color', [0.10 0.10 0.10], 'FontSize', 9, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1], 'Margin', 2, 'HandleVisibility', 'off');

lgd = legend(ax_grid, [h_grid_curr, h_grid_pred, h_grid_actual, h_grid_tail], ...
       {'Current', 'Predicted Next', 'Actual Next', 'Recent'}, 'Location', 'best');
set(lgd, 'AutoUpdate', 'off');

% Context/global limits
if ~AUTO_ZOOM
    seg_xmin = min(samples(:,1)); seg_xmax = max(samples(:,1));
    seg_ymin = min(samples(:,2)); seg_ymax = max(samples(:,2));
    cx = 0.5 * (seg_xmin + seg_xmax);
    cy = 0.5 * (seg_ymin + seg_ymax);

    raw_xmin = cx - CONTEXT_RADIUS_M; raw_xmax = cx + CONTEXT_RADIUS_M;
    raw_ymin = cy - CONTEXT_RADIUS_M; raw_ymax = cy + CONTEXT_RADIUS_M;

    xlim(ax_raw, [raw_xmin, raw_xmax]);
    ylim(ax_raw, [raw_ymin, raw_ymax]);

    % Use full simulation topology extent for right panel, so it is not clipped.
    if ~isempty(topo_model)
        [bx, by] = boundary(topo_model.road_poly);
        gx_topo = bx ./ GRID_SIZE + 1;
        gy_topo = by ./ GRID_SIZE + 1;
        gx_min = floor(min(gx_topo)) - GRID_MARGIN_CELLS;
        gx_max = ceil(max(gx_topo)) + GRID_MARGIN_CELLS;
        gy_min = floor(min(gy_topo)) - GRID_MARGIN_CELLS;
        gy_max = ceil(max(gy_topo)) + GRID_MARGIN_CELLS;
    elseif ~isempty(topology_xy)
        topo_grid = floor(topology_xy ./ GRID_SIZE) + 1;
        gx_min = min(topo_grid(:,1)) - GRID_MARGIN_CELLS;
        gx_max = max(topo_grid(:,1)) + GRID_MARGIN_CELLS;
        gy_min = min(topo_grid(:,2)) - GRID_MARGIN_CELLS;
        gy_max = max(topo_grid(:,2)) + GRID_MARGIN_CELLS;
    else
        gx_min = min(grid_seq(:,1)) - 6;
        gx_max = max(grid_seq(:,1)) + 6;
        gy_min = min(grid_seq(:,2)) - 6;
        gy_max = max(grid_seq(:,2)) + 6;
    end

    xlim(ax_grid, [gx_min, gx_max]);
    ylim(ax_grid, [gy_min, gy_max]);
end

if AUTO_ZOOM
    gx_min = min(grid_seq(:,1)) - 5;
    gx_max = max(grid_seq(:,1)) + 5;
    gy_min = min(grid_seq(:,2)) - 5;
    gy_max = max(grid_seq(:,2)) + 5;
end

draw_grid_mesh(ax_grid, gx_min, gx_max, gy_min, gy_max, GRID_MESH_STEP);

start_idx = max(WINDOW, 6);
end_idx = size(grid_seq, 1) - 1;
hits = 0;
total = 0;

dirs = [0 1; 1 1; 1 0; 1 -1; 0 -1; -1 -1; -1 0; -1 1; 0 0];

for i = start_idx:STEP_STRIDE:end_idx
    cur_gx = grid_seq(i, 1);
    cur_gy = grid_seq(i, 2);

    recent_grid = grid_seq((i - 5):i, :);
    recent_raw = samples((i - 5):i, :);

    dx = mean(diff(recent_grid(:, 1)));
    dy = mean(diff(recent_grid(:, 2)));
    if dx == 0 && dy == 0
        continue;
    end

    heading = mod(atan2(dx, dy) * 180 / pi, 360);
    lane = lane_from_heading(heading);
    gid = int32((cur_gy - 1) * 2000 + (cur_gx - 1));

    used_mode = 'INERTIA';
    max_dir = heading_to_direction(heading);

    if isKey(db{lane}, gid)
        probs = db{lane}(gid);
        if abs(dx) > 0 || abs(dy) > 0
            probs(9) = 0;
        end
        [~, max_dir] = max(probs);
        used_mode = 'DB';
    end

    delta = dirs(max_dir, :);
    pred_gx = cur_gx + delta(1);
    pred_gy = cur_gy + delta(2);
    actual_gx = grid_seq(i + 1, 1);
    actual_gy = grid_seq(i + 1, 2);

    jump = abs(actual_gx - cur_gx) + abs(actual_gy - cur_gy);
    if jump > 2
        continue;
    end

    total = total + 1;
    is_hit = (pred_gx == actual_gx) && (pred_gy == actual_gy);
    if is_hit
        hits = hits + 1;
    end

    % Convert to raw meter coordinates
    pred_x_m = (pred_gx - 0.5) * GRID_SIZE;
    pred_y_m = (pred_gy - 0.5) * GRID_SIZE;
    cur_x_m = samples(i, 1);
    cur_y_m = samples(i, 2);
    actual_x_m = samples(i + 1, 1);
    actual_y_m = samples(i + 1, 2);

    % Build moving car body orientation from raw motion
    raw_dx = samples(i + 1, 1) - samples(i, 1);
    raw_dy = samples(i + 1, 2) - samples(i, 2);
    theta = atan2(raw_dy, raw_dx);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    car_world = (R * car_shape_local')';
    car_world(:,1) = car_world(:,1) + cur_x_m;
    car_world(:,2) = car_world(:,2) + cur_y_m;

    dir_norm = max(hypot(raw_dx, raw_dy), 1e-6);
    hx = (raw_dx / dir_norm) * 6.0;
    hy = (raw_dy / dir_norm) * 6.0;

    % Update raw panel
    set(h_raw_tail, 'XData', recent_raw(:,1), 'YData', recent_raw(:,2));
    set(h_raw_pred, 'XData', pred_x_m, 'YData', pred_y_m);
    set(h_raw_actual, 'XData', actual_x_m, 'YData', actual_y_m);
    set(h_raw_car, 'XData', car_world(:,1), 'YData', car_world(:,2));
    set(h_raw_head, 'XData', cur_x_m, 'YData', cur_y_m, 'UData', hx, 'VData', hy);

    if AUTO_ZOOM
        xlim(ax_raw, [cur_x_m - 150, cur_x_m + 150]);
        ylim(ax_raw, [cur_y_m - 150, cur_y_m + 150]);
    end

    % Update grid panel
    set(h_grid_tail, 'XData', recent_grid(:,1), 'YData', recent_grid(:,2));
    set(h_grid_curr, 'XData', cur_gx, 'YData', cur_gy);
    set(h_grid_pred, 'XData', pred_gx, 'YData', pred_gy);
    set(h_grid_actual, 'XData', actual_gx, 'YData', actual_gy);
    set(h_grid_link_pred, 'XData', [cur_gx pred_gx], 'YData', [cur_gy pred_gy]);
    set(h_grid_link_actual, 'XData', [cur_gx actual_gx], 'YData', [cur_gy actual_gy]);
    set(h_grid_text, 'Position', [cur_gx + 0.7, cur_gy + 0.7, 0], ...
        'String', sprintf('C(%d,%d)  P(%d,%d)  A(%d,%d)', ...
        cur_gx, cur_gy, pred_gx, pred_gy, actual_gx, actual_gy));

    if AUTO_ZOOM
        xlim(ax_grid, [cur_gx - 18, cur_gx + 18]);
        ylim(ax_grid, [cur_gy - 18, cur_gy + 18]);
    end

    status = 'MISS';
    ttl_color = [0.75 0 0];
    if is_hit
        status = 'HIT';
        ttl_color = [0 0.5 0];
    end

    ttl = sprintf(['Seg=%d/%d (%s) | Step=%d | Mode=%s | %s | ', ...
                   'C=(%d,%d) P=(%d,%d) A=(%d,%d) | Acc=%.2f%%'], ...
                  idx, numel(tracks), track.scenario, ...
                  i, used_mode, status, ...
                  cur_gx, cur_gy, pred_gx, pred_gy, actual_gx, actual_gy, ...
                  100 * hits / max(total, 1));
    title(tl, ttl, 'Color', ttl_color, 'Interpreter', 'none', 'FontSize', 11);

    drawnow;
    pause(PAUSE_SEC);
end

fprintf('Visualization done.\n');
fprintf('Evaluated steps: %d\n', total);
fprintf('Exact hit count: %d\n', hits);
fprintf('Exact accuracy:  %.2f%%\n', 100 * hits / max(total, 1));


function draw_grid_mesh(ax, gx_min, gx_max, gy_min, gy_max, step)
    for gx = gx_min:step:gx_max
        plot(ax, [gx gx], [gy_min gy_max], '-', ...
            'Color', [1.00 0.25 0.25], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
    for gy = gy_min:step:gy_max
        plot(ax, [gx_min gx_max], [gy gy], '-', ...
            'Color', [1.00 0.25 0.25], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
end

function draw_topology_grid(ax, topo_model, grid_size)
    % Draw road polygon boundary in grid coordinates.
    [bx, by] = boundary(topo_model.road_poly);
    gx = bx ./ grid_size + 1;
    gy = by ./ grid_size + 1;
    plot(ax, gx, gy, '-', 'Color', [0.60 0.60 0.60], ...
        'LineWidth', 1.4, 'HandleVisibility', 'off');

    % Draw lane markings in grid coordinates.
    for mk = 1:numel(topo_model.lane_markings)
        lm = topo_model.lane_markings{mk};
        lgx = lm(:,1) ./ grid_size + 1;
        lgy = lm(:,2) ./ grid_size + 1;
        plot(ax, lgx, lgy, '--', 'Color', [0.55 0.55 0.55], ...
            'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end

function xy = load_topology_points(csv_path)
    fid = fopen(csv_path, 'r');
    if fid < 0
        xy = zeros(0, 2);
        return;
    end

    header = fgetl(fid); %#ok<NASGU>
    data = textscan(fid, '%s %f %f %f', 'Delimiter', ',');
    fclose(fid);

    x = data{3};
    y = data{4};
    mask = ~isnan(x) & ~isnan(y);
    xy = [x(mask), y(mask)];
end

function tracks = load_test_tracks(fname)
    fid = fopen(fname, 'r');
    if fid < 0
        error('Could not open test file: %s', fname);
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

