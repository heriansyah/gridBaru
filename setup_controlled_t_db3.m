clearvars -except TEST_DIR TEST_MANEUVER;
clc;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
GRID = 10.0;
DB_DIR = fullfile(SCRIPT_DIR, 'DB_T3');
CUSTOM_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories_t3.csv');
TEST_FILE = fullfile(SCRIPT_DIR, 'test_generated_t3.csv');

% T approaches: S, W, E
if ~exist('TEST_DIR', 'var')
    TEST_DIR = 'S';
end
if ~exist('TEST_MANEUVER', 'var')
    TEST_MANEUVER = 'left';
end
TEST_NAME = sprintf('Test_t_%s_to_%s', upper(TEST_DIR), lower(TEST_MANEUVER));

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

% Straight approach states
for gy = -20:-1
    lane_maps{1}(gid_of(1, gy)) = onehot(1); % S->center (northbound)
end
for gx = -20:-1
    lane_maps{2}(gid_of(gx, 1)) = onehot(3); % W->center (eastbound)
end
for gx = 2:21
    lane_maps{4}(gid_of(gx, 1)) = onehot(7); % E->center (westbound)
end

% Outgoing support states
for gx = 2:15
    lane_maps{2}(gid_of(gx, 1)) = onehot(3); % to east
end
for gx = -15:-1
    lane_maps{4}(gid_of(gx, 1)) = onehot(7); % to west
end
for gy = -15:-1
    lane_maps{3}(gid_of(1, gy)) = onehot(5); % to south
end

% Intersection probabilities at center cell (1,1), no U-turn, no closed arm to north.
g0 = gid_of(1, 1);
% From south: left->W(7), right->E(3)
lane_maps{1}(g0) = normalize_probs(dir_probs(7, 0.60, 3, 0.40, 9, 0.00));
% From west: straight->E(3), right->S(5)
lane_maps{2}(g0) = normalize_probs(dir_probs(3, 0.70, 5, 0.30, 9, 0.00));
% From east: straight->W(7), left->S(5)
lane_maps{4}(g0) = normalize_probs(dir_probs(7, 0.65, 5, 0.35, 9, 0.00));

% Write DB lane files
for lane = 1:4
    write_lane_db(fullfile(DB_DIR, sprintf('dbx_lane%d.csv', lane)), lane_maps{lane});
end

% Create per-vehicle DB folders
veh_ids = {'S', 'W', 'E'};
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
    struct('name', 'Train_t_S_to_left',   'dir', 'S', 'maneuver', 'left'), ...
    struct('name', 'Train_t_W_to_straight','dir', 'W', 'maneuver', 'straight'), ...
    struct('name', 'Train_t_E_to_straight','dir', 'E', 'maneuver', 'straight'), ...
    struct('name', TEST_NAME, 'dir', upper(TEST_DIR), 'maneuver', lower(TEST_MANEUVER))
};

write_custom_and_test(segments, GRID, CUSTOM_FILE, TEST_FILE);

% Per-direction test files
DIR_SET = {'S', 'W', 'E'};
for d = 1:numel(DIR_SET)
    dir_d = DIR_SET{d};
    man_d = default_test_maneuver_t(dir_d);
    name_d = sprintf('Test_t_%s_to_%s', dir_d, man_d);
    file_d = fullfile(SCRIPT_DIR, sprintf('test_generated_t3_%s.csv', dir_d));
    write_single_test(file_d, name_d, dir_d, man_d, GRID);
end

fprintf('=== CONTROLLED T SETUP DONE ===\n');
fprintf('DB: %s\n', DB_DIR);
fprintf('custom: %s\n', CUSTOM_FILE);
fprintf('test: %s\n', TEST_FILE);
fprintf('Active test scenario: %s | dir=%s | maneuver=%s\n', TEST_NAME, upper(TEST_DIR), lower(TEST_MANEUVER));
fprintf('Lane states: L1=%d, L2=%d, L3=%d, L4=%d\n', ...
    lane_maps{1}.Count, lane_maps{2}.Count, lane_maps{3}.Count, lane_maps{4}.Count);
fprintf('Per-vehicle DB folders created: veh_S, veh_W, veh_E\n');


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

function write_custom_and_test(segments, grid, custom_file, test_file)
fid = fopen(custom_file, 'w');
if fid < 0, error('Cannot write %s', custom_file); end
fprintf(fid, 'scenario,timestamp,x,y\n');

fid_test = fopen(test_file, 'w');
if fid_test < 0
    fclose(fid); error('Cannot write %s', test_file);
end

for s = 1:numel(segments)
    [x, y] = make_route_t_xy(segments{s}.dir, segments{s}.maneuver, grid);
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

function write_single_test(file_d, name_d, dir_d, man_d, grid)
fid = fopen(file_d, 'w');
if fid < 0, error('Cannot write %s', file_d); end
[x, y] = make_route_t_xy(dir_d, man_d, grid);
for t = 1:numel(x)
    fprintf(fid, '%s,%.3f,%.3f\n', name_d, x(t), y(t));
end
fprintf(fid, 'SPLIT,NaN,NaN\n');
fclose(fid);
end

function [x, y] = make_route_t_xy(dir_in, maneuver, grid)
n_in = 20; n_out = 20;
switch upper(dir_in)
    case 'S' % from south to center (northbound)
        x_in = zeros(1,n_in); y_in = -grid*(n_in:-1:1); x0 = 0; y0 = 0;
        switch lower(maneuver)
            case 'right', x_out = grid*(1:n_out); y_out = zeros(1,n_out);  % east
            otherwise,    x_out = -grid*(1:n_out); y_out = zeros(1,n_out); % left->west
        end
    case 'W' % from west to center (eastbound)
        x_in = -grid*(n_in:-1:1); y_in = zeros(1,n_in); x0 = 0; y0 = 0;
        if strcmpi(maneuver, 'right')
            x_out = zeros(1,n_out); y_out = -grid*(1:n_out);               % south
        else
            x_out = grid*(1:n_out); y_out = zeros(1,n_out);                % straight east
        end
    otherwise % 'E' from east to center (westbound)
        x_in = grid*(n_in:-1:1); y_in = zeros(1,n_in); x0 = 0; y0 = 0;
        if strcmpi(maneuver, 'left')
            x_out = zeros(1,n_out); y_out = -grid*(1:n_out);               % south
        else
            x_out = -grid*(1:n_out); y_out = zeros(1,n_out);               % straight west
        end
end
x = [x_in, x0, x_out];
y = [y_in, y0, y_out];
end

function maneuver = default_test_maneuver_t(dir_in)
switch upper(dir_in)
    case 'S', maneuver = 'left';
    case 'W', maneuver = 'straight';
    otherwise, maneuver = 'straight'; % E
end
end
