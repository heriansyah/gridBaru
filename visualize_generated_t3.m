clear;
clc;
close all;

% ------------------------------------------------------------
% Visualize / animate T-intersection generated trajectories.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories_t3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_T3');
PAUSE_SEC = 0.04;
TAIL_LEN = 18;
PRED_DIST_M = 40;
TURN_ANGLE_THRESH_DEG = 35;
INERTIA_ALPHA = 0.45;
LANE_SHIFT_REF_M = 1.2;
SLOW_GAIN = 1.0;
DRIFT_GAIN = 0.6;

if ~exist(CSV_FILE, 'file')
    error('File not found: %s. Run generate_custom_data_t3.m first.', CSV_FILE);
end

tracks = load_tracks_by_scenario(CSV_FILE);
if isempty(tracks)
    error('No scenarios found in %s', CSV_FILE);
end

topo = topology_t3();
db = [];
if exist(DB_FOLDER, 'dir') == 7
    db = load_database(DB_FOLDER);
end

fig = figure('Name', 'T-Intersection Generated Trajectories', ...
             'NumberTitle', 'off', ...
             'Color', 'w', ...
             'Position', [120 80 1100 680]);

ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
plot(topo.road_poly, 'FaceColor', [0.92 0.92 0.92], 'EdgeColor', [0.25 0.25 0.25]);
for k = 1:numel(topo.lane_markings)
    lm = topo.lane_markings{k};
    plot(ax, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
end

car_length = 4.4;
car_width = 1.8;
car_shape_local = [
    -car_width/2, -car_length/2;
     car_width/2, -car_length/2;
     car_width/2,  car_length/2;
    -car_width/2,  car_length/2
];

h_tail = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.8);
h_car = patch(ax, NaN, NaN, [0.1 0.35 0.9], 'FaceAlpha', 0.9, 'EdgeColor', [0 0 0], 'LineWidth', 1.0);
h_head = quiver(ax, NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
h_pred = plot(ax, NaN, NaN, 'rx', 'LineWidth', 2.2, 'MarkerSize', 11);
h_actual = plot(ax, NaN, NaN, 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

lgd = legend(ax, [h_tail, h_pred, h_actual], {'Trajectory', 'Predicted', 'Actual'}, 'Location', 'best');
set(lgd, 'AutoUpdate', 'off');

xlim(ax, topo.bounds(1:2));
ylim(ax, topo.bounds(3:4));
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');

for s = 1:numel(tracks)
    x = tracks{s}.xy(:, 1);
    y = tracks{s}.xy(:, 2);
    name = tracks{s}.scenario;

    [pred_xy, actual_xy] = compute_turn_markers( ...
        x, y, db, PRED_DIST_M, TURN_ANGLE_THRESH_DEG, ...
        INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN);
    if ~isempty(pred_xy)
        set(h_pred, 'XData', pred_xy(1), 'YData', pred_xy(2));
    else
        set(h_pred, 'XData', NaN, 'YData', NaN);
    end
    if ~isempty(actual_xy)
        set(h_actual, 'XData', actual_xy(1), 'YData', actual_xy(2));
    else
        set(h_actual, 'XData', NaN, 'YData', NaN);
    end

    for i = 1:numel(x)
        s0 = max(1, i - TAIL_LEN);
        set(h_tail, 'XData', x(s0:i), 'YData', y(s0:i));

        if i < numel(x)
            dx = x(i+1) - x(i);
            dy = y(i+1) - y(i);
        else
            dx = x(i) - x(i-1);
            dy = y(i) - y(i-1);
        end

        theta = atan2(dy, dx);
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        car_world = (R * car_shape_local')';
        car_world(:,1) = car_world(:,1) + x(i);
        car_world(:,2) = car_world(:,2) + y(i);

        set(h_car, 'XData', car_world(:,1), 'YData', car_world(:,2));

        dir_norm = max(hypot(dx, dy), 1e-6);
        ux = dx / dir_norm;
        uy = dy / dir_norm;
        set(h_head, 'XData', x(i), 'YData', y(i), 'UData', ux * 6, 'VData', uy * 6);

        title(ax, sprintf('Scenario %d/%d | %s', s, numel(tracks), name), 'Interpreter', 'none');
        drawnow;
        pause(PAUSE_SEC);
    end
end

disp('Visualization complete.');


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

function [pred_xy, actual_xy] = compute_turn_markers( ...
    x, y, db, pred_dist, turn_thresh_deg, inertia_alpha, lane_shift_ref, slow_gain, drift_gain)
    pred_xy = [];
    actual_xy = [];

    if numel(x) < 8
        return;
    end

    d = hypot(x, y);
    [~, idx0] = min(d);

    s = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];
    s0 = s(idx0);
    [~, idx_target] = min(abs(s - (s0 + pred_dist)));
    actual_xy = [x(idx_target), y(idx_target)];

    if isempty(db)
        return;
    end

    i1 = max(1, idx0 - 8);
    i2 = min(numel(x), idx0);
    v_in = mean_vec(x, y, i1, i2);
    if any(isnan(v_in))
        return;
    end

    heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
    lane = lane_from_heading(heading);

    gx = floor(x(idx0) / 10.0) + 1;
    gy = floor(y(idx0) / 10.0) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));

    turn = predict_turn_with_inertia( ...
        db{lane}, gid, lane, x, y, idx0, v_in, ...
        inertia_alpha, lane_shift_ref, slow_gain, drift_gain);
    if isempty(turn)
        return;
    end

    u_out = turn_to_vector(lane, turn);
    center = [x(idx0), y(idx0)];
    pred_xy = center + u_out * pred_dist;
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

function u = turn_to_vector(lane, turn)
    switch lane
        case 1 % moving north
            if strcmp(turn, 'left'), u = [-1, 0];
            elseif strcmp(turn, 'right'), u = [1, 0];
            else, u = [0, 1];
            end
        case 2 % moving east
            if strcmp(turn, 'left'), u = [0, 1];
            elseif strcmp(turn, 'right'), u = [0, -1];
            else, u = [1, 0];
            end
        case 3 % moving south
            if strcmp(turn, 'left'), u = [1, 0];
            elseif strcmp(turn, 'right'), u = [-1, 0];
            else, u = [0, -1];
            end
        otherwise % lane 4 moving west
            if strcmp(turn, 'left'), u = [0, -1];
            elseif strcmp(turn, 'right'), u = [0, 1];
            else, u = [-1, 0];
            end
    end
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

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function v = clamp_value(v, vmin, vmax)
    v = max(vmin, min(vmax, v));
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
