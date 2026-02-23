clearvars -except TEST_DIR TEST_MANEUVER;
clc;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
GRID = 10.0;
STEM_CELLS = 22;    % 220 m (matches topology_y3 stem length)
BRANCH_CELLS = 17;  % 170 m (matches topology_y3 branch tips)
DB_DIR = fullfile(SCRIPT_DIR, 'DB_Y3');
CUSTOM_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories_y3.csv');
TEST_FILE = fullfile(SCRIPT_DIR, 'test_generated_y3.csv');

% Y approaches: S (stem), NW branch, NE branch
if ~exist('TEST_DIR', 'var')
    TEST_DIR = 'S';
end
if ~exist('TEST_MANEUVER', 'var')
    TEST_MANEUVER = 'left';
end
TEST_NAME = sprintf('Test_y_%s_to_%s', upper(TEST_DIR), lower(TEST_MANEUVER));

if ~exist(DB_DIR, 'dir')
    mkdir(DB_DIR);
end

lane_maps = {
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any')
};

gid_of = @(gx, gy) int32((gy - 1) * 2000 + (gx - 1));

% Approaches in grid indices around center (1,1):
% S->center: (1,-20..-1), direction N (lane1)
for gy = -STEM_CELLS:-1
    lane_maps{1}(gid_of(1, gy)) = onehot(1);
end
% NW->center: diagonal SE (lane3), start near (-17,17) to (0,0)
for k = 1:BRANCH_CELLS
    gx = -BRANCH_CELLS + k;
    gy = BRANCH_CELLS + 1 - k;
    lane_maps{3}(gid_of(gx, gy)) = onehot(4); % SE
end
% NE->center: diagonal SW (lane4), start near (17,17) to (0,0)
for k = 1:BRANCH_CELLS
    gx = BRANCH_CELLS + 1 - k;
    gy = BRANCH_CELLS + 1 - k;
    lane_maps{4}(gid_of(gx, gy)) = onehot(6); % SW
end

% Outgoing supports:
% center->NE (dir2, lane2)
for k = 1:BRANCH_CELLS
    lane_maps{2}(gid_of(1 + k, 1 + k)) = onehot(2);
end
% center->NW (dir8, lane4)
for k = 1:BRANCH_CELLS
    lane_maps{4}(gid_of(1 - k, 1 + k)) = onehot(8);
end
% center->S (dir5, lane3)
for gy = -STEM_CELLS:-1
    lane_maps{3}(gid_of(1, gy)) = onehot(5);
end

% Split/intersection probabilities at center cell g0
g0 = gid_of(1, 1);
% From stem (lane1): left->NW(8), right->NE(2)
lane_maps{1}(g0) = normalize_probs(dir_probs(8, 0.60, 2, 0.40, 9, 0.00));
% From NW branch inbound (lane3): straight-ish->NE(2), right->S(5)
lane_maps{3}(g0) = normalize_probs(dir_probs(2, 0.70, 5, 0.30, 9, 0.00));
% From NE branch inbound (lane4): straight-ish->NW(8), left->S(5)
lane_maps{4}(g0) = normalize_probs(dir_probs(8, 0.65, 5, 0.35, 9, 0.00));

% Write DB files
for lane = 1:4
    write_lane_db(fullfile(DB_DIR, sprintf('dbx_lane%d.csv', lane)), lane_maps{lane});
end

% Create per-vehicle DB folders
veh_ids = {'S', 'NW', 'NE'};
for v = 1:numel(veh_ids)
    vdir = fullfile(DB_DIR, ['veh_', veh_ids{v}]);
    if ~exist(vdir, 'dir')
        mkdir(vdir);
    end
    for lane = 1:4
        copyfile(fullfile(DB_DIR, sprintf('dbx_lane%d.csv', lane)), ...
                 fullfile(vdir, sprintf('dbx_lane%d.csv', lane)));
    end
end

segments = {
    struct('name', 'Train_y_S_to_left',   'dir', 'S',  'maneuver', 'left'), ...
    struct('name', 'Train_y_NW_to_straight','dir', 'NW', 'maneuver', 'straight'), ...
    struct('name', 'Train_y_NE_to_straight','dir', 'NE', 'maneuver', 'straight'), ...
    struct('name', TEST_NAME, 'dir', upper(TEST_DIR), 'maneuver', lower(TEST_MANEUVER))
};

write_custom_and_test(segments, GRID, CUSTOM_FILE, TEST_FILE, STEM_CELLS, BRANCH_CELLS);

% Per-direction test files
DIR_SET = {'S', 'NW', 'NE'};
for d = 1:numel(DIR_SET)
    dir_d = DIR_SET{d};
    man_d = default_test_maneuver_y(dir_d);
    name_d = sprintf('Test_y_%s_to_%s', dir_d, man_d);
    file_d = fullfile(SCRIPT_DIR, sprintf('test_generated_y3_%s.csv', dir_d));
    write_single_test(file_d, name_d, dir_d, man_d, GRID, STEM_CELLS, BRANCH_CELLS);
end

fprintf('=== CONTROLLED Y SETUP DONE ===\n');
fprintf('DB: %s\n', DB_DIR);
fprintf('custom: %s\n', CUSTOM_FILE);
fprintf('test: %s\n', TEST_FILE);
fprintf('Active test scenario: %s | dir=%s | maneuver=%s\n', TEST_NAME, upper(TEST_DIR), lower(TEST_MANEUVER));
fprintf('Lane states: L1=%d, L2=%d, L3=%d, L4=%d\n', ...
    lane_maps{1}.Count, lane_maps{2}.Count, lane_maps{3}.Count, lane_maps{4}.Count);
fprintf('Per-vehicle DB folders created: veh_S, veh_NW, veh_NE\n');


function p = onehot(idx)
p = zeros(1,9); p(idx) = 1.0;
end

function p = dir_probs(i1, v1, i2, v2, i3, v3)
p = zeros(1,9);
p(i1)=p(i1)+v1; p(i2)=p(i2)+v2; p(i3)=p(i3)+v3;
end

function p = normalize_probs(p)
s = sum(p); if s > 0, p = p / s; end
end

function write_lane_db(fname, m)
fid = fopen(fname, 'w');
if fid < 0, error('Cannot write DB file: %s', fname); end
ks = m.keys; ks_num = sort(cell2mat(ks));
for i = 1:numel(ks_num)
    gid = int32(ks_num(i));
    probs = m(gid);
    fprintf(fid, '%d', gid);
    for k = 1:9, fprintf(fid, ',%.6f', probs(k)); end
    fprintf(fid, '\n');
end
fclose(fid);
end

function write_custom_and_test(segments, grid, custom_file, test_file, stem_cells, branch_cells)
fid = fopen(custom_file, 'w');
if fid < 0, error('Cannot write %s', custom_file); end
fprintf(fid, 'scenario,timestamp,x,y\n');

fid_test = fopen(test_file, 'w');
if fid_test < 0
    fclose(fid); error('Cannot write %s', test_file);
end

for s = 1:numel(segments)
    [x, y] = make_route_y_xy(segments{s}.dir, segments{s}.maneuver, grid, stem_cells, branch_cells);
    for t = 1:numel(x)
        fprintf(fid, '%s,%d,%.3f,%.3f\n', segments{s}.name, t, x(t), y(t));
    end
    if startsWith(segments{s}.name, 'Test_')
        for t = 1:numel(x)
            fprintf(fid_test, '%s,%.3f,%.3f\n', segments{s}.name, x(t), y(t));
        end
        fprintf(fid_test, 'SPLIT,NaN,NaN\n');
    end
end
fclose(fid); fclose(fid_test);
end

function write_single_test(file_d, name_d, dir_d, man_d, grid, stem_cells, branch_cells)
fid = fopen(file_d, 'w');
if fid < 0, error('Cannot write %s', file_d); end
[x, y] = make_route_y_xy(dir_d, man_d, grid, stem_cells, branch_cells);
for t = 1:numel(x)
    fprintf(fid, '%s,%.3f,%.3f\n', name_d, x(t), y(t));
end
fprintf(fid, 'SPLIT,NaN,NaN\n');
fclose(fid);
end

function [x, y] = make_route_y_xy(dir_in, maneuver, grid, stem_cells, branch_cells)
switch upper(dir_in)
    case 'S' % stem to split
        x_in = zeros(1,stem_cells); y_in = -grid*(stem_cells:-1:1); x0=0; y0=0;
        if strcmpi(maneuver, 'right')
            x_out = grid*(1:branch_cells); y_out = grid*(1:branch_cells);    % to NE
        else
            x_out = -grid*(1:branch_cells); y_out = grid*(1:branch_cells);   % to NW (left/default)
        end
    case 'NW' % NW branch to split (SE)
        x_in = -grid*(branch_cells:-1:1); y_in = grid*(branch_cells:-1:1); x0=0; y0=0;
        if strcmpi(maneuver, 'right')
            x_out = zeros(1,stem_cells); y_out = -grid*(1:stem_cells);   % to S
        else
            x_out = grid*(1:branch_cells); y_out = grid*(1:branch_cells);    % straight-ish to NE
        end
    otherwise % 'NE' branch to split (SW)
        x_in = grid*(branch_cells:-1:1); y_in = grid*(branch_cells:-1:1); x0=0; y0=0;
        if strcmpi(maneuver, 'left')
            x_out = zeros(1,stem_cells); y_out = -grid*(1:stem_cells);   % to S
        else
            x_out = -grid*(1:branch_cells); y_out = grid*(1:branch_cells);   % straight-ish to NW
        end
end
x = [x_in, x0, x_out];
y = [y_in, y0, y_out];
end

function maneuver = default_test_maneuver_y(dir_in)
switch upper(dir_in)
    case 'S', maneuver = 'left';
    case 'NW', maneuver = 'straight';
    otherwise, maneuver = 'straight'; % NE
end
end
