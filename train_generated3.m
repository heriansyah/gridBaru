clear;
clc;

fprintf('=== TRAINING ON GENERATED DATA (SMARTGRID3) ===\n\n');

SCRIPT_DIR = fileparts(mfilename('fullpath'));
INPUT_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Generated3');
TEST_FILE = fullfile(SCRIPT_DIR, 'test_generated3.csv');
GRID_SIZE = 10.0;
STRAIGHT_DOMINANCE_MIN = 0.85;
BRANCH_SECOND_MIN = 0.15;
MIN_OBS_FOR_SHARPEN = 5;
INTERSECTION_SECOND_MIN = 0.12;
MIN_OBS_FOR_INTERSECTION = 4;
INTERSECTION_LEFT_P = 0.60;
INTERSECTION_STRAIGHT_P = 0.30;
INTERSECTION_RIGHT_P = 0.10;

if ~exist(INPUT_FILE, 'file')
    error('Input file not found: %s', INPUT_FILE);
end

if ~exist(DB_FOLDER, 'dir')
    mkdir(DB_FOLDER);
end

fprintf('Reading %s...\n', INPUT_FILE);
fid = fopen(INPUT_FILE, 'r');
if fid < 0
    error('Could not open input file: %s', INPUT_FILE);
end

header = fgetl(fid); %#ok<NASGU>
data = textscan(fid, '%s %f %f %f', 'Delimiter', ',', 'CollectOutput', false);
fclose(fid);

scenarios = data{1};
xs = data{3};
ys = data{4};

if isempty(scenarios)
    error('No data rows found in %s', INPUT_FILE);
end

valid_mask = ~isnan(xs) & ~isnan(ys);
scenarios = scenarios(valid_mask);
xs = xs(valid_mask);
ys = ys(valid_mask);

unique_scenarios = unique(scenarios);
num_scenarios = numel(unique_scenarios);
fprintf('Total scenarios: %d\n', num_scenarios);

has_labels = any(contains(unique_scenarios, 'Test_'));
if has_labels
    fprintf('Detected labels. Using explicit Train/Test split.\n');
    test_mask = contains(unique_scenarios, 'Test_');
    test_scenarios = unique_scenarios(test_mask);
    train_scenarios = unique_scenarios(~test_mask);
else
    fprintf('No labels detected. Using random 80/20 split.\n');
    rng(42);
    idx = randperm(num_scenarios);
    split_point = floor(0.8 * num_scenarios);
    train_scenarios = unique_scenarios(idx(1:split_point));
    test_scenarios = unique_scenarios(idx(split_point + 1:end));
end

fprintf('Training scenarios: %d\n', numel(train_scenarios));
fprintf('Testing scenarios:  %d\n\n', numel(test_scenarios));

if isempty(train_scenarios)
    error('No training scenarios available.');
end
if isempty(test_scenarios)
    warning('No test scenarios found. test_generated3.csv will be empty except separators.');
end

lane_maps = {
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any')
};

transitions = 0;

for s = 1:numel(train_scenarios)
    sid = train_scenarios{s};
    mask = strcmp(scenarios, sid);
    s_xs = xs(mask);
    s_ys = ys(mask);

    prev_gx = NaN;
    prev_gy = NaN;

    for i = 1:numel(s_xs)
        gx = floor(s_xs(i) / GRID_SIZE) + 1;
        gy = floor(s_ys(i) / GRID_SIZE) + 1;

        if isnan(prev_gx)
            prev_gx = gx;
            prev_gy = gy;
            continue;
        end

        if gx == prev_gx && gy == prev_gy
            continue;
        end

        dx = gx - prev_gx;
        dy = gy - prev_gy;

        % Compass heading: 0=N, 90=E, 180=S, 270=W
        heading = mod(atan2(dx, dy) * 180 / pi, 360);

        if heading >= 315 || heading < 45
            lane = 1;
        elseif heading >= 45 && heading < 135
            lane = 2;
        elseif heading >= 135 && heading < 225
            lane = 3;
        else
            lane = 4;
        end

        c = direction_index(dx, dy);

        gid = int32((prev_gy - 1) * 2000 + (prev_gx - 1));
        m = lane_maps{lane};
        if ~isKey(m, gid)
            m(gid) = zeros(1, 9);
        end

        counts = m(gid);
        counts(c) = counts(c) + 1;
        m(gid) = counts;

        transitions = transitions + 1;
        prev_gx = gx;
        prev_gy = gy;
    end
end

fprintf('Transitions learned: %d\n', transitions);
if transitions == 0
    error('No transitions learned. Check parse output and GRID_SIZE.');
end

max_prob_sum_error = 0.0;
written_rows = 0;
sharpened_rows = 0;
branched_rows = 0;
forced_intersection_rows = 0;

for lane = 1:4
    fname = fullfile(DB_FOLDER, sprintf('dbx_lane%d.csv', lane));
    fid = fopen(fname, 'w');
    if fid < 0
        error('Cannot write database file: %s', fname);
    end

    m = lane_maps{lane};
    ks = m.keys;

    for k = 1:numel(ks)
        gid = ks{k};
        counts = m(gid);
        total = sum(counts);
        if total <= 0
            continue;
        end

        probs = counts / total;

        % Make non-intersection cells deterministic (one dominant direction),
        % while forcing intersection cells to 3 options:
        % left / straight / right (no U-turn).
        moving_probs = probs(1:8);
        sorted_moving = sort(moving_probs, 'descend');
        p1 = sorted_moving(1);
        p2 = sorted_moving(2);
        is_intersection = (total >= MIN_OBS_FOR_INTERSECTION) && (p2 >= INTERSECTION_SECOND_MIN);

        if is_intersection
            probs = force_intersection_probs( ...
                lane, INTERSECTION_LEFT_P, INTERSECTION_STRAIGHT_P, INTERSECTION_RIGHT_P);
            forced_intersection_rows = forced_intersection_rows + 1;
            branched_rows = branched_rows + 1;
        elseif total >= MIN_OBS_FOR_SHARPEN && p1 >= STRAIGHT_DOMINANCE_MIN && p2 < BRANCH_SECOND_MIN
            hard = zeros(1, 9);
            [~, imax] = max(probs);
            hard(imax) = 1.0;
            probs = hard;
            sharpened_rows = sharpened_rows + 1;
        else
            branched_rows = branched_rows + 1;
        end

        sum_err = abs(sum(probs) - 1.0);
        if sum_err > max_prob_sum_error
            max_prob_sum_error = sum_err;
        end

        fprintf(fid, '%d', gid);
        for p = 1:9
            fprintf(fid, ',%.6f', probs(p));
        end
        fprintf(fid, '\n');
        written_rows = written_rows + 1;
    end

    fclose(fid);
end

fprintf('Database saved to %s\n', DB_FOLDER);
fprintf('DB rows written: %d\n', written_rows);
fprintf('Max |sum(prob)-1|: %.12f\n', max_prob_sum_error);
fprintf('Sharpened (straight-like) rows: %d\n', sharpened_rows);
fprintf('Branched (intersection-like) rows: %d\n', branched_rows);
fprintf('Forced 3-way intersection rows: %d\n', forced_intersection_rows);

fid = fopen(TEST_FILE, 'w');
if fid < 0
    error('Cannot write test file: %s', TEST_FILE);
end

for s = 1:numel(test_scenarios)
    sid = test_scenarios{s};
    mask = strcmp(scenarios, sid);
    s_xs = xs(mask);
    s_ys = ys(mask);

    for i = 1:numel(s_xs)
        fprintf(fid, '%s,%.3f,%.3f\n', sid, s_xs(i), s_ys(i));
    end
    fprintf(fid, 'SPLIT,NaN,NaN\n');
end

fclose(fid);
fprintf('Test data saved to %s\n', TEST_FILE);


function probs = force_intersection_probs(lane, left_p, straight_p, right_p)
    probs = zeros(1, 9);

    [left_idx, straight_idx, right_idx] = lane_turn_indices(lane);

    probs(left_idx) = left_p;
    probs(straight_idx) = straight_p;
    probs(right_idx) = right_p;
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

function idx = direction_index(dx, dy)
    sx = sign(dx);
    sy = sign(dy);

    if sy == 1 && sx == 0
        idx = 1;  % N
    elseif sy == 1 && sx == 1
        idx = 2;  % NE
    elseif sy == 0 && sx == 1
        idx = 3;  % E
    elseif sy == -1 && sx == 1
        idx = 4;  % SE
    elseif sy == -1 && sx == 0
        idx = 5;  % S
    elseif sy == -1 && sx == -1
        idx = 6;  % SW
    elseif sy == 0 && sx == -1
        idx = 7;  % W
    elseif sy == 1 && sx == -1
        idx = 8;  % NW
    else
        idx = 9;  % Stay
    end
end



