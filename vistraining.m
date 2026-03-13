clear;
clc;
close all;

% ------------------------------------------------------------
% Visualize training knowledge growth from custom_trajectories3.csv
% Outputs:
% 1) Snapshot heatmaps (knowledge mask grows as training increases)
% 2) Growth curve (known grid cells vs number of training scenarios)
% 3) Optional frame-by-frame PNG per training scenario
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');
STEP_DIR = fullfile(FIG_DIR, 'vistraining_steps');

GRID_SIZE = 10.0;
MAX_SNAPSHOTS = 6;
SAVE_EACH_TRAINING_FRAME = true;

if ~exist(CSV_FILE, 'file')
    error('File not found: %s', CSV_FILE);
end
if ~exist(FIG_DIR, 'dir')
    mkdir(FIG_DIR);
end
if SAVE_EACH_TRAINING_FRAME && ~exist(STEP_DIR, 'dir')
    mkdir(STEP_DIR);
end

fid = fopen(CSV_FILE, 'r');
if fid < 0
    error('Cannot open %s', CSV_FILE);
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
    error('No valid rows in %s', CSV_FILE);
end

uniq = unique(scenarios, 'stable');
is_test = cellfun(@(s) strncmp(s, 'Test_', 5), uniq);
train_ids = uniq(~is_test);
n_train = numel(train_ids);

if n_train == 0
    error('No training scenarios detected. Expected scenario names not starting with "Test_".');
end

is_train_row = ~cellfun(@(s) strncmp(s, 'Test_', 5), scenarios);
x_train = xs(is_train_row);
y_train = ys(is_train_row);

gx_all = floor(x_train ./ GRID_SIZE) + 1;
gy_all = floor(y_train ./ GRID_SIZE) + 1;

gx_min = min(gx_all);
gx_max = max(gx_all);
gy_min = min(gy_all);
gy_max = max(gy_all);

W = gx_max - gx_min + 1;
H = gy_max - gy_min + 1;

knowledge_count = zeros(H, W);
known_cells_each = zeros(n_train, 1);
obs_each = zeros(n_train, 1);

snapshot_steps = choose_snapshot_steps(n_train, MAX_SNAPSHOTS);
snapshot_masks = cell(numel(snapshot_steps), 1);
snapshot_known = zeros(numel(snapshot_steps), 1);

% Blue-yellow style similar to common "knowledge matrix" examples.
cmap_knowledge = [0.15 0.15 0.75; 0.98 0.92 0.10];

if SAVE_EACH_TRAINING_FRAME
    fig_step = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 700 520]);
    ax_step = axes(fig_step);
    h_img_step = imagesc(ax_step, gx_min:gx_max, gy_min:gy_max, zeros(H, W));
    axis(ax_step, 'xy');
    axis(ax_step, 'equal');
    axis(ax_step, 'tight');
    grid(ax_step, 'on');
    box(ax_step, 'on');
    xlabel(ax_step, 'Grid X');
    ylabel(ax_step, 'Grid Y');
    colormap(ax_step, cmap_knowledge);
    caxis(ax_step, [0 1]);
end

next_snap_idx = 1;
for i = 1:n_train
    sid = train_ids{i};
    m = strcmp(scenarios, sid);
    gx = floor(xs(m) ./ GRID_SIZE) + 1;
    gy = floor(ys(m) ./ GRID_SIZE) + 1;

    cx = gx - gx_min + 1;
    cy = gy - gy_min + 1;

    inside = (cx >= 1) & (cx <= W) & (cy >= 1) & (cy <= H);
    cx = cx(inside);
    cy = cy(inside);

    if ~isempty(cx)
        lin = sub2ind([H, W], cy, cx);
        add = accumarray(lin, 1, [H * W, 1]);
        knowledge_count = knowledge_count + reshape(add, [H, W]);
    end

    known_mask = knowledge_count > 0;
    known_cells_each(i) = nnz(known_mask);
    obs_each(i) = sum(knowledge_count(:));

    if next_snap_idx <= numel(snapshot_steps) && i == snapshot_steps(next_snap_idx)
        snapshot_masks{next_snap_idx} = known_mask;
        snapshot_known(next_snap_idx) = known_cells_each(i);
        next_snap_idx = next_snap_idx + 1;
    end

    if SAVE_EACH_TRAINING_FRAME
        set(h_img_step, 'CData', known_mask);
        title(ax_step, sprintf('Knowledge after training %d/%d', i, n_train), 'FontWeight', 'bold');
        drawnow;
        out_step = fullfile(STEP_DIR, sprintf('vistraining_step_%03d.png', i));
        print(fig_step, out_step, '-dpng', '-r200');
    end
end

if SAVE_EACH_TRAINING_FRAME
    close(fig_step);
end

% Snapshot figure for paper.
n_snap = numel(snapshot_steps);
n_cols = min(3, n_snap);
n_rows = ceil(n_snap / n_cols);

fig_snap = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1500 760]);
tiledlayout(fig_snap, n_rows, n_cols, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:n_snap
    ax = nexttile;
    imagesc(ax, gx_min:gx_max, gy_min:gy_max, snapshot_masks{k});
    axis(ax, 'xy');
    axis(ax, 'equal');
    axis(ax, 'tight');
    grid(ax, 'on');
    box(ax, 'on');
    colormap(ax, cmap_knowledge);
    caxis(ax, [0 1]);
    xlabel(ax, 'Grid X');
    ylabel(ax, 'Grid Y');
    title(ax, sprintf('Train %d/%d | Known Cells: %d', ...
        snapshot_steps(k), n_train, snapshot_known(k)), 'FontWeight', 'bold');
end

sgtitle(fig_snap, 'Knowledge Growth from Training Data', 'FontWeight', 'bold');
print(fig_snap, fullfile(FIG_DIR, 'vistraining_growth_snapshots.png'), '-dpng', '-r220');
print(fig_snap, fullfile(FIG_DIR, 'vistraining_growth_snapshots.pdf'), '-dpdf', '-bestfit');
close(fig_snap);

% Growth curve figure for paper text/table support.
fig_curve = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 450]);
yyaxis left;
plot(1:n_train, known_cells_each, 'b-o', 'LineWidth', 1.8, 'MarkerSize', 4, ...
    'MarkerFaceColor', [0.15 0.45 0.90]);
ylabel('Known Grid Cells');
ylim([0, max(known_cells_each) * 1.08 + 1]);

yyaxis right;
plot(1:n_train, obs_each, 'Color', [0.90 0.35 0.10], 'LineWidth', 1.6);
ylabel('Accumulated Observations');

xlabel('Training Scenario Index');
title('Knowledge Growth During Training');
grid on;
box on;
legend({'Known Cells', 'Observations'}, 'Location', 'northwest');

print(fig_curve, fullfile(FIG_DIR, 'vistraining_growth_curve.png'), '-dpng', '-r220');
print(fig_curve, fullfile(FIG_DIR, 'vistraining_growth_curve.pdf'), '-dpdf', '-bestfit');
close(fig_curve);

fprintf('vistraining complete.\n');
fprintf('Training scenarios: %d\n', n_train);
fprintf('Grid bounds gx=[%d..%d], gy=[%d..%d]\n', gx_min, gx_max, gy_min, gy_max);
fprintf('Final known cells: %d\n', known_cells_each(end));
fprintf('Figures saved:\n');
fprintf('- %s\n', fullfile(FIG_DIR, 'vistraining_growth_snapshots.png'));
fprintf('- %s\n', fullfile(FIG_DIR, 'vistraining_growth_snapshots.pdf'));
fprintf('- %s\n', fullfile(FIG_DIR, 'vistraining_growth_curve.png'));
fprintf('- %s\n', fullfile(FIG_DIR, 'vistraining_growth_curve.pdf'));
if SAVE_EACH_TRAINING_FRAME
    fprintf('- step frames: %s\n', STEP_DIR);
end


function steps = choose_snapshot_steps(n_train, max_snapshots)
    if n_train <= max_snapshots
        steps = 1:n_train;
    else
        steps = unique(round(linspace(1, n_train, max_snapshots)));
        if steps(end) ~= n_train
            steps(end + 1) = n_train;
        end
    end
end
