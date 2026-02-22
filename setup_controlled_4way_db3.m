clearvars -except TEST_DIR TEST_MANEUVER;
clc;

% Controlled 4-way setup:
% - Keep simple deterministic straight-road DB (100% straight).
% - One intersection cell with lane-specific probabilities.
% - Build test/custom trajectories with one vehicle per approach.

SCRIPT_DIR = fileparts(mfilename('fullpath'));
GRID = 10.0;
DB_DIR = fullfile(SCRIPT_DIR, 'DB_Generated3');
CUSTOM_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
TEST_FILE = fullfile(SCRIPT_DIR, 'test_generated3.csv');

% Optional CLI-style overrides (set these vars before run):
%   TEST_DIR = 'N'|'S'|'E'|'W'
%   TEST_MANEUVER = 'left'|'right'|'straight'
if ~exist('TEST_DIR', 'var')
    TEST_DIR = 'E';
end
if ~exist('TEST_MANEUVER', 'var')
    TEST_MANEUVER = 'right';
end
TEST_NAME = sprintf('Test_4way_%s_to_%s', upper(TEST_DIR), lower(TEST_MANEUVER));

if ~exist(DB_DIR, 'dir')
    mkdir(DB_DIR);
end

% Direction index:
% 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW, 9=Stay

% -------------------------------------------------------------------------
% 1) Build DB (lane1..lane4)
% -------------------------------------------------------------------------
lane_maps = {
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any'), ...
    containers.Map('KeyType', 'int32', 'ValueType', 'any')
};

% Helper for gid from grid cell index (gx,gy).
gid_of = @(gx, gy) int32((gy - 1) * 2000 + (gx - 1));

% Straight-road one-hot states on 4 approaches (20 cells each).
% Center intersection cell in this setup is (gx,gy)=(1,1), i.e. around (x,y)=(0,0).
for gy = -20:-1
    lane_maps{1}(gid_of(1, gy)) = onehot(1);  % lane1 (northbound) straight N
end
for gy = 2:21
    lane_maps{3}(gid_of(1, gy)) = onehot(5);  % lane3 (southbound) straight S
end
for gx = -20:-1
    lane_maps{2}(gid_of(gx, 1)) = onehot(3);  % lane2 (eastbound) straight E
end
for gx = 2:21
    lane_maps{4}(gid_of(gx, 1)) = onehot(7);  % lane4 (westbound) straight W
end

% Add a little outgoing straight support after intersection.
for gx = 2:15
    lane_maps{2}(gid_of(gx, 1)) = onehot(3);
end
for gx = -15:-1
    lane_maps{4}(gid_of(gx, 1)) = onehot(7);
end
for gy = 2:15
    lane_maps{1}(gid_of(1, gy)) = onehot(1);
end
for gy = -15:-1
    lane_maps{3}(gid_of(1, gy)) = onehot(5);
end

% Intersection cell probabilities (no U-turn), one cell only.
g0 = gid_of(1, 1);

% 1) Northbound: right 60, straight 30, left 10
% facing north => right=E(3), straight=N(1), left=W(7)
lane_maps{1}(g0) = normalize_probs(dir_probs(3, 0.60, 1, 0.30, 7, 0.10));

% 2) Southbound: left 70, straight 20, right 10
% facing south => left=E(3), straight=S(5), right=W(7)
lane_maps{3}(g0) = normalize_probs(dir_probs(3, 0.70, 5, 0.20, 7, 0.10));

% 3) Westbound: straight 65, right 20, left 15
% facing west => straight=W(7), right=N(1), left=S(5)
lane_maps{4}(g0) = normalize_probs(dir_probs(7, 0.65, 1, 0.20, 5, 0.15));

% 4) Eastbound: right 55, left 35, straight 20 (sum=1.10 => normalized)
% facing east => right=S(5), left=N(1), straight=E(3)
lane_maps{2}(g0) = normalize_probs(dir_probs(5, 0.55, 1, 0.35, 3, 0.20));

% Write DB files.
for lane = 1:4
    fname = fullfile(DB_DIR, sprintf('dbx_lane%d.csv', lane));
    fid = fopen(fname, 'w');
    if fid < 0
        error('Cannot write DB file: %s', fname);
    end

    m = lane_maps{lane};
    ks = m.keys;
    ks_num = cell2mat(ks);
    ks_num = sort(ks_num);

    for i = 1:numel(ks_num)
        gid = int32(ks_num(i));
        probs = m(gid);
        fprintf(fid, '%d', gid);
        for k = 1:9
            fprintf(fid, ',%.6f', probs(k));
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
end

% -------------------------------------------------------------------------
% 2) Build simple trajectories (one per approach) for custom + test
% -------------------------------------------------------------------------
segments = {
    struct('name', 'Train_4way_N_to_right', 'dir', 'N', 'maneuver', 'right'), ...
    struct('name', 'Train_4way_S_to_left',  'dir', 'S', 'maneuver', 'left'), ...
    struct('name', 'Train_4way_W_straight', 'dir', 'W', 'maneuver', 'straight'), ...
    struct('name', TEST_NAME, 'dir', upper(TEST_DIR), 'maneuver', lower(TEST_MANEUVER))
};

fid = fopen(CUSTOM_FILE, 'w');
if fid < 0
    error('Cannot write %s', CUSTOM_FILE);
end
fprintf(fid, 'scenario,timestamp,x,y\n');

fid_test = fopen(TEST_FILE, 'w');
if fid_test < 0
    fclose(fid);
    error('Cannot write %s', TEST_FILE);
end

for s = 1:numel(segments)
    [x, y] = make_route_xy(segments{s}.dir, segments{s}.maneuver, GRID);
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

fclose(fid);
fclose(fid_test);

fprintf('=== CONTROLLED 4-WAY SETUP DONE ===\n');
fprintf('DB: %s\n', DB_DIR);
fprintf('custom: %s\n', CUSTOM_FILE);
fprintf('test: %s\n', TEST_FILE);
fprintf('Active test scenario: %s | dir=%s | maneuver=%s\n', ...
    TEST_NAME, upper(TEST_DIR), lower(TEST_MANEUVER));
fprintf('Lane states: L1=%d, L2=%d, L3=%d, L4=%d\n', ...
    lane_maps{1}.Count, lane_maps{2}.Count, lane_maps{3}.Count, lane_maps{4}.Count);
fprintf('\nRun next:\n');
fprintf('  mygrid3(''predict'', ''%s'', 0, 15, 10)\n', TEST_FILE);


function p = onehot(idx)
    p = zeros(1,9);
    p(idx) = 1.0;
end

function p = dir_probs(i1, v1, i2, v2, i3, v3)
    p = zeros(1,9);
    p(i1) = p(i1) + v1;
    p(i2) = p(i2) + v2;
    p(i3) = p(i3) + v3;
end

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function [x, y] = make_route_xy(dir_in, maneuver, grid)
    n_in = 20;
    n_out = 20;

    switch upper(dir_in)
        case 'N'  % from south to north
            x_in = zeros(1, n_in);
            y_in = -grid * (n_in:-1:1);
            x0 = 0; y0 = 0;
            switch lower(maneuver)
                case 'right', x_out = grid * (1:n_out); y_out = zeros(1,n_out);   % east
                case 'left',  x_out = -grid * (1:n_out); y_out = zeros(1,n_out);  % west
                otherwise,    x_out = zeros(1,n_out); y_out = grid * (1:n_out);   % north
            end

        case 'S'  % from north to south
            x_in = zeros(1, n_in);
            y_in = grid * (n_in:-1:1);
            x0 = 0; y0 = 0;
            switch lower(maneuver)
                case 'right', x_out = -grid * (1:n_out); y_out = zeros(1,n_out);  % west
                case 'left',  x_out = grid * (1:n_out); y_out = zeros(1,n_out);   % east
                otherwise,    x_out = zeros(1,n_out); y_out = -grid * (1:n_out);  % south
            end

        case 'E'  % from west to east
            x_in = -grid * (n_in:-1:1);
            y_in = zeros(1, n_in);
            x0 = 0; y0 = 0;
            switch lower(maneuver)
                case 'right', x_out = zeros(1,n_out); y_out = -grid * (1:n_out);  % south
                case 'left',  x_out = zeros(1,n_out); y_out = grid * (1:n_out);   % north
                otherwise,    x_out = grid * (1:n_out); y_out = zeros(1,n_out);   % east
            end

        otherwise % 'W' from east to west
            x_in = grid * (n_in:-1:1);
            y_in = zeros(1, n_in);
            x0 = 0; y0 = 0;
            switch lower(maneuver)
                case 'right', x_out = zeros(1,n_out); y_out = grid * (1:n_out);   % north
                case 'left',  x_out = zeros(1,n_out); y_out = -grid * (1:n_out);  % south
                otherwise,    x_out = -grid * (1:n_out); y_out = zeros(1,n_out);  % west
            end
    end

    x = [x_in, x0, x_out];
    y = [y_in, y0, y_out];
end
