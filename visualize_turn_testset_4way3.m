clear;
% Keep previous eval statistics visible in Command Window.
% clc;
close all;

% ------------------------------------------------------------
% Animate all turn test datasets sequentially in one figure.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'test_intersection_4way3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Intersection4way3');
PAUSE_SEC = 0.04;
TAIL_LEN = 18;
PRED_DIST_M = 40;  % distance ahead from center to visualize predicted turn
TURN_ANGLE_THRESH_DEG = 35;
INERTIA_ALPHA = 0.7;  % 0..1, higher => stronger bias to motion cues
LANE_SHIFT_REF_M = 1.2;
SLOW_GAIN = 1.0;
DRIFT_GAIN = 0.6;
PAUSE_PER_SCENARIO = false; % autoplay all scenarios when called from eval

if ~exist(CSV_FILE, 'file')
    error('File not found: %s. Run generate_turn_testset_4way3.m first.', CSV_FILE);
end

tracks = load_tracks_by_scenario(CSV_FILE);
if isempty(tracks)
    error('No scenarios found in %s', CSV_FILE);
end

topo = topology_4way3();
db = [];
if exist(DB_FOLDER, 'dir') == 7
    db = load_database(DB_FOLDER);
end

fig = figure('Name', '4-way Turn Testset Animation', ...
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
h_info = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontSize', 9, ...
    'BackgroundColor', [1 1 1], 'Margin', 6, 'EdgeColor', [0.6 0.6 0.6]);

xlim(ax, topo.bounds(1:2));
ylim(ax, topo.bounds(3:4));
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');

for s = 1:numel(tracks)
    x = tracks{s}.xy(:, 1);
    y = tracks{s}.xy(:, 2);
    name = tracks{s}.scenario;

    % Compute predicted/actual turn marker once per scenario
    [pred_xy, actual_xy, info] = compute_turn_markers( ...
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
    set(h_info, 'String', build_info_text(info));

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

    if PAUSE_PER_SCENARIO
        title(ax, sprintf('Scenario %d/%d | %s | press any key to continue', ...
            s, numel(tracks), name), 'Interpreter', 'none');
        drawnow;
        waitforbuttonpress;
    else
        pause(0.3);
    end
end

disp('All scenarios animated.');


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

    % Preserve order of appearance
    [uniq, first_idx] = unique(scenarios, 'stable');
    tracks = cell(numel(uniq), 1);
    for i = 1:numel(uniq)
        sid = uniq{i};
        mask = strcmp(scenarios, sid);
        tracks{i} = struct('scenario', sid, 'xy', [xs(mask), ys(mask)]);
    end
end

function [pred_xy, actual_xy, info] = compute_turn_markers( ...
    x, y, db, pred_dist, turn_thresh_deg, inertia_alpha, lane_shift_ref, slow_gain, drift_gain)
    pred_xy = [];
    actual_xy = [];
    info = struct('lane', NaN, 'heading', NaN, 'gid', NaN, ...
        'probs', [], 'probs_final', [], 'alpha', inertia_alpha, ...
        'speed_ratio', NaN, 'lat_delta', NaN, 'slow_w', NaN, 'drift_w', NaN, ...
        'pred', '', 'actual', '', 'note', '');

    if numel(x) < 8
        info.note = 'Too few points';
        return;
    end

    d = hypot(x, y);
    [~, idx0] = min(d);

    % actual: find point at ~pred_dist after center along path
    s = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];
    s0 = s(idx0);
    [~, idx_target] = min(abs(s - (s0 + pred_dist)));
    actual_xy = [x(idx_target), y(idx_target)];

    % actual turn (from geometry)
    i1 = max(1, idx0 - 8);
    i2 = min(numel(x), idx0 + 8);
    v_in = mean_vec(x, y, i1, idx0);
    v_out = mean_vec(x, y, idx0, i2);
    if ~any(isnan(v_in)) && ~any(isnan(v_out))
        info.actual = turn_from_vectors(v_in, v_out, turn_thresh_deg);
    end

    if isempty(db)
        info.note = 'DB not loaded';
        return;
    end

    % compute incoming heading using a small window before center
    i1 = max(1, idx0 - 8);
    i2 = min(numel(x), idx0);
    v_in = mean_vec(x, y, i1, i2);
    if any(isnan(v_in))
        info.note = 'No valid heading';
        return;
    end

    heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
    lane = lane_from_heading(heading);
    info.heading = heading;
    info.lane = lane;

    gx = floor(x(idx0) / 10.0) + 1;
    gy = floor(y(idx0) / 10.0) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));
    info.gid = gid;

    [turn, probs3] = predict_turn_from_db(db{lane}, gid, lane);
    info.probs = probs3;
    if isempty(turn)
        info.note = 'GID not in DB';
        return;
    end

    % Motion cues: speed drop and lateral drift near intersection.
    [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in);
    info.speed_ratio = speed_ratio;
    info.lat_delta = lat_delta;

    slow_w = clamp_value((1 - speed_ratio) * slow_gain, 0, 0.6);
    drift_w = clamp_value((abs(lat_delta) / max(lane_shift_ref, 1e-6)) * drift_gain, 0, 0.6);
    info.slow_w = slow_w;
    info.drift_w = drift_w;

    p_bias = [0.5 * slow_w, 1 - slow_w, 0.5 * slow_w];
    if lat_delta > 0
        p_bias = [p_bias(1) + drift_w, p_bias(2), max(p_bias(3) - drift_w, 0)];
    elseif lat_delta < 0
        p_bias = [max(p_bias(1) - drift_w, 0), p_bias(2), p_bias(3) + drift_w];
    end
    p_bias = normalize_probs(p_bias);

    p_final = normalize_probs((1 - inertia_alpha) * probs3 + inertia_alpha * p_bias);
    info.probs_final = p_final;

    % choose predicted turn after inertia
    [~, imax] = max(p_final);
    if imax == 1
        turn_final = 'left';
    elseif imax == 2
        turn_final = 'straight';
    else
        turn_final = 'right';
    end
    info.pred = turn_final;

    u_out = turn_to_vector(lane, turn_final);
    center = [x(idx0), y(idx0)];
    pred_xy = center + u_out * pred_dist;
end

function txt = build_info_text(info)
    line1 = sprintf('lane=%s | heading=%.1f° | gid=%s', ...
        num_to_str(info.lane), info.heading, num_to_str(info.gid));
    if isempty(info.probs)
        line2 = 'pBase: L=NA S=NA R=NA';
        line3 = 'pFinal: L=NA S=NA R=NA | pred=NA';
    else
        line2 = sprintf('pBase: L=%.2f S=%.2f R=%.2f', ...
            info.probs(1), info.probs(2), info.probs(3));
        if isempty(info.probs_final)
            line3 = sprintf('pFinal: L=NA S=NA R=NA | pred=%s', str_or_na(info.pred));
        else
            line3 = sprintf('pFinal: L=%.2f S=%.2f R=%.2f | pred=%s', ...
                info.probs_final(1), info.probs_final(2), info.probs_final(3), str_or_na(info.pred));
        end
    end
    line4 = sprintf('actual=%s | alpha=%.2f', str_or_na(info.actual), info.alpha);
    line5 = sprintf('speedRatio=%.2f | latDelta=%.2f | slow=%.2f drift=%.2f', ...
        info.speed_ratio, info.lat_delta, info.slow_w, info.drift_w);
    if isempty(info.note)
        txt = sprintf('%s\n%s\n%s\n%s\n%s', line1, line2, line3, line4, line5);
    else
        txt = sprintf('%s\n%s\n%s\n%s\n%s\nnote: %s', line1, line2, line3, line4, line5, info.note);
    end
end

function s = num_to_str(v)
    if isempty(v) || any(isnan(v))
        s = 'NA';
    else
        s = sprintf('%d', v);
    end
end

function s = str_or_na(v)
    if isempty(v)
        s = 'NA';
    else
        s = v;
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

function [pred, probs3] = predict_turn_from_db(map, gid, lane)
    pred = '';
    probs3 = [];
    if ~isKey(map, gid)
        return;
    end
    probs = map(gid);
    [left_idx, straight_idx, right_idx] = lane_turn_indices(lane);
    p = [probs(left_idx), probs(straight_idx), probs(right_idx)];
    probs3 = p;
    [~, imax] = max(p);
    if imax == 1
        pred = 'left';
    elseif imax == 2
        pred = 'straight';
    else
        pred = 'right';
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

function [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in)
    n = numel(x);
    if n < 6 || idx0 < 3 || idx0 > n - 2
        speed_ratio = 1.0;
        lat_delta = 0.0;
        return;
    end

    % speed ratio: center vs before
    i_before1 = max(1, idx0 - 8);
    i_before2 = max(2, idx0 - 1);
    i_center1 = max(1, idx0 - 2);
    i_center2 = min(n, idx0 + 2);
    sp_before = mean(hypot(diff(x(i_before1:i_before2)), diff(y(i_before1:i_before2))));
    sp_center = mean(hypot(diff(x(i_center1:i_center2)), diff(y(i_center1:i_center2))));
    speed_ratio = sp_center / max(sp_before, 1e-6);

    % lateral drift relative to incoming direction
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
