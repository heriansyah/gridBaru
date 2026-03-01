clear;
clc;

% ------------------------------------------------------------
% Intersection-only training for 4-way.
% Focus: left / straight / right decisions at the intersection,
% not coordinate-level accuracy.
% ------------------------------------------------------------
SCRIPT_DIR = fileparts(mfilename('fullpath'));
INPUT_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Intersection4way3');
TURN_CSV = fullfile(SCRIPT_DIR, 'intersection_turns_4way3.csv');

GRID_SIZE = 10.0;
INTERSECTION_RADIUS_M = 25;     % points within this radius are "intersection"
PRE_POST_WINDOW = 12;           % points before/after closest-to-center sample
TURN_ANGLE_THRESH_DEG = 35;     % |angle| <= thresh => straight
USE_SCENARIO_LABELS = true;     % if scenario names include left/right/straight
MIN_POINTS = 8;

if ~exist(INPUT_FILE, 'file')
    error('Input file not found: %s', INPUT_FILE);
end

if ~exist(DB_FOLDER, 'dir')
    mkdir(DB_FOLDER);
end

fprintf('=== TRAIN INTERSECTION-ONLY (4-WAY) ===\n');
fprintf('Input: %s\n', INPUT_FILE);
fprintf('DB out: %s\n', DB_FOLDER);
fprintf('Intersection radius: %.1f m\n', INTERSECTION_RADIUS_M);
fprintf('Heading window: %d\n', PRE_POST_WINDOW);
fprintf('Angle thresh: %.1f deg\n\n', TURN_ANGLE_THRESH_DEG);

fid = fopen(INPUT_FILE, 'r');
if fid < 0
    error('Cannot open %s', INPUT_FILE);
end
header = fgetl(fid); %#ok<NASGU>
data = textscan(fid, '%s %f %f %f', 'Delimiter', ',', 'CollectOutput', false);
fclose(fid);

scenarios = data{1};
xs = data{3};
ys = data{4};

valid_mask = ~isnan(xs) & ~isnan(ys);
scenarios = scenarios(valid_mask);
xs = xs(valid_mask);
ys = ys(valid_mask);

if isempty(scenarios)
    error('No valid data rows in %s', INPUT_FILE);
end

unique_scenarios = unique(scenarios);
has_labels = any(contains(unique_scenarios, 'Test_'));
if has_labels
    train_scenarios = unique_scenarios(~contains(unique_scenarios, 'Test_'));
else
    train_scenarios = unique_scenarios;
end

fprintf('Scenarios total: %d\n', numel(unique_scenarios));
fprintf('Scenarios used for training: %d\n\n', numel(train_scenarios));

% turn_counts(lane, turn): turn order = [left, straight, right]
turn_counts = zeros(4, 3);

% For each lane, collect intersection cells to write DB rows.
cell_maps = {
    containers.Map('KeyType', 'int32', 'ValueType', 'logical'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'logical'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'logical'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'logical')
};

% Write labeled turn table for inspection.
fid_turn = fopen(TURN_CSV, 'w');
if fid_turn < 0
    error('Cannot write %s', TURN_CSV);
end
fprintf(fid_turn, 'scenario,lane,turn\n');

skipped = 0;
for s = 1:numel(train_scenarios)
    sid = train_scenarios{s};
    mask = strcmp(scenarios, sid);
    x = xs(mask);
    y = ys(mask);
    n = numel(x);

    if n < MIN_POINTS
        skipped = skipped + 1;
        continue;
    end

    d = hypot(x, y);
    [~, idx0] = min(d);

    i1 = max(1, idx0 - PRE_POST_WINDOW);
    i2 = min(n, idx0 + PRE_POST_WINDOW);

    v_in = mean_vec(x, y, i1, idx0);
    v_out = mean_vec(x, y, idx0, i2);
    if any(isnan(v_in)) || any(isnan(v_out))
        skipped = skipped + 1;
        continue;
    end

    heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
    lane = lane_from_heading(heading);

    turn = '';
    if USE_SCENARIO_LABELS
        turn = turn_from_label(sid);
    end
    if isempty(turn)
        turn = turn_from_vectors(v_in, v_out, TURN_ANGLE_THRESH_DEG);
    end

    if isempty(turn)
        skipped = skipped + 1;
        continue;
    end

    fprintf(fid_turn, '%s,%d,%s\n', sid, lane, turn);

    t_idx = turn_to_index(turn);
    turn_counts(lane, t_idx) = turn_counts(lane, t_idx) + 1;

    % Collect intersection grid cells for this lane.
    int_mask = d <= INTERSECTION_RADIUS_M;
    if any(int_mask)
        gx = floor(x(int_mask) ./ GRID_SIZE) + 1;
        gy = floor(y(int_mask) ./ GRID_SIZE) + 1;
        gid = int32((gy - 1) * 2000 + (gx - 1));
        m = cell_maps{lane};
        for k = 1:numel(gid)
            m(gid(k)) = true;
        end
        cell_maps{lane} = m;
    end
end

fclose(fid_turn);

% Build DB with per-lane turn probabilities at intersection cells.
written_rows = 0;
for lane = 1:4
    counts = turn_counts(lane, :);
    total = sum(counts);
    if total == 0
        fprintf('Warning: lane %d has no turn samples. Using 1/3 each.\n', lane);
        probs3 = [1/3, 1/3, 1/3];
    else
        probs3 = counts / total;
    end

    [left_idx, straight_idx, right_idx] = lane_turn_indices(lane);
    p9 = zeros(1, 9);
    p9(left_idx) = probs3(1);
    p9(straight_idx) = probs3(2);
    p9(right_idx) = probs3(3);

    fname = fullfile(DB_FOLDER, sprintf('dbx_lane%d.csv', lane));
    fid = fopen(fname, 'w');
    if fid < 0
        error('Cannot write DB file: %s', fname);
    end

    m = cell_maps{lane};
    ks = m.keys;
    ks_num = sort(cell2mat(ks));
    for i = 1:numel(ks_num)
        gid = int32(ks_num(i));
        fprintf(fid, '%d', gid);
        for k = 1:9
            fprintf(fid, ',%.6f', p9(k));
        end
        fprintf(fid, '\n');
        written_rows = written_rows + 1;
    end

    fclose(fid);
end

fprintf('\nTurn counts (left/straight/right) per lane:\n');
for lane = 1:4
    fprintf('Lane %d: %d / %d / %d\n', lane, turn_counts(lane, 1), turn_counts(lane, 2), turn_counts(lane, 3));
end
fprintf('Intersection DB rows written: %d\n', written_rows);
fprintf('Turn labels saved: %s\n', TURN_CSV);
fprintf('DB saved to: %s\n', DB_FOLDER);
fprintf('Skipped scenarios: %d\n', skipped);


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
    % Direction index:
    % 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW, 9=Stay
    if lane == 1
        % Moving north
        left_idx = 7;      % W
        straight_idx = 1;  % N
        right_idx = 3;     % E
    elseif lane == 2
        % Moving east
        left_idx = 1;      % N
        straight_idx = 3;  % E
        right_idx = 5;     % S
    elseif lane == 3
        % Moving south
        left_idx = 3;      % E
        straight_idx = 5;  % S
        right_idx = 7;     % W
    else
        % Moving west (lane 4)
        left_idx = 5;      % S
        straight_idx = 7;  % W
        right_idx = 1;     % N
    end
end
