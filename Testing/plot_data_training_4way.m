clear;
clc;
close all;

% ------------------------------------------------------------
% Plot data training 4-way:
% 1) Perpindahan posisi (trajectory)
% 2) Kecepatan (profile vs waktu + ringkasan mean/std)
%
% Input  : custom_trajectories3.csv
% Output :
% - figs/training_position_4way.png
% - figs/training_position_4way.pdf
% - figs/training_speed_4way.png
% - figs/training_speed_4way.pdf
% - figs/training_speed_4way_summary.csv
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');

if ~exist(CSV_FILE, 'file')
    error('File tidak ditemukan: %s', CSV_FILE);
end
if ~exist(FIG_DIR, 'dir')
    mkdir(FIG_DIR);
end

T = readtable(CSV_FILE, 'TextType', 'string');
must_cols = ["scenario", "timestamp", "x", "y"];
if ~all(ismember(must_cols, string(T.Properties.VariableNames)))
    error('CSV wajib punya kolom: scenario,timestamp,x,y');
end

% Ambil data training saja.
T = T(startsWith(T.scenario, "Train_"), :);
if isempty(T)
    error('Tidak ada data training di %s', CSV_FILE);
end

scenario_ids = unique(T.scenario, 'stable');
n_scen = numel(scenario_ids);

% Grup untuk warna/legend.
group_names = {'left', 'right', 'straight-normal', 'straight-cautious', 'straight-redlight', 'other'};
group_colors = [
    0.20 0.50 0.85;  % left
    0.85 0.35 0.15;  % right
    0.25 0.60 0.25;  % straight-normal
    0.55 0.45 0.75;  % straight-cautious
    0.85 0.20 0.20;  % straight-redlight
    0.45 0.45 0.45   % other
];
n_groups = numel(group_names);

% Simpan per-skenario.
traj_x = cell(n_scen, 1);
traj_y = cell(n_scen, 1);
time_cells = cell(n_scen, 1);
speed_cells = cell(n_scen, 1);
scen_group = zeros(n_scen, 1);

for i = 1:n_scen
    sid = scenario_ids(i);
    m = (T.scenario == sid);

    ts = T.timestamp(m);
    xs = T.x(m);
    ys = T.y(m);

    [ts, ord] = sort(ts);
    xs = xs(ord);
    ys = ys(ord);

    traj_x{i} = xs;
    traj_y{i} = ys;

    dt = diff(ts);
    ds = hypot(diff(xs), diff(ys));
    vi = ds ./ max(dt, 1e-6);
    ti = ts(2:end);

    time_cells{i} = ti(:);
    speed_cells{i} = vi(:);
    scen_group(i) = classify_group(sid);
end

% -----------------------------
% Plot 1: perpindahan posisi
% -----------------------------
fig_pos = figure('Visible', 'off', 'Color', 'w', 'Position', [90 90 1050 760]);
ax_pos = axes(fig_pos);
hold(ax_pos, 'on');
grid(ax_pos, 'on');
axis(ax_pos, 'equal');
box(ax_pos, 'on');
xlabel(ax_pos, 'X (m)');
ylabel(ax_pos, 'Y (m)');
title(ax_pos, 'Perpindahan Posisi Data Training (4-Way)');

for i = 1:n_scen
    g = scen_group(i);
    c = group_colors(g, :);
    plot(ax_pos, traj_x{i}, traj_y{i}, '-', 'Color', c, 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
end

% Dummy plot untuk legend ringkas.
for g = 1:n_groups
    plot(ax_pos, NaN, NaN, '-', 'Color', group_colors(g, :), 'LineWidth', 2.0, ...
        'DisplayName', group_names{g});
end
legend(ax_pos, 'Location', 'eastoutside');

print(fig_pos, fullfile(FIG_DIR, 'training_position_4way.png'), '-dpng', '-r220');
print(fig_pos, fullfile(FIG_DIR, 'training_position_4way.pdf'), '-dpdf', '-bestfit');
close(fig_pos);

% -----------------------------
% Plot 2: kecepatan
% -----------------------------
t_all = unique(cell2mat(time_cells));
t_all = t_all(:)';
n_t = numel(t_all);
Vmat = NaN(n_scen, n_t);
for i = 1:n_scen
    ti = time_cells{i};
    vi = speed_cells{i};
    [tf, loc] = ismember(ti, t_all);
    Vmat(i, loc(tf)) = vi(tf);
end

group_counts = zeros(n_groups, 1);
group_mean_curve = NaN(n_groups, n_t);
group_mean_speed = NaN(n_groups, 1);
group_std_speed = NaN(n_groups, 1);

for g = 1:n_groups
    idx = (scen_group == g);
    group_counts(g) = sum(idx);
    if any(idx)
        group_mean_curve(g, :) = mean(Vmat(idx, :), 1, 'omitnan');
        vals = Vmat(idx, :);
        vals = vals(~isnan(vals));
        group_mean_speed(g) = mean(vals);
        group_std_speed(g) = std(vals);
    end
end

fig_spd = figure('Visible', 'off', 'Color', 'w', 'Position', [90 70 1100 760]);
tiledlayout(fig_spd, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile;
hold(ax1, 'on');
grid(ax1, 'on');
box(ax1, 'on');
for i = 1:n_scen
    g = scen_group(i);
    c_light = 0.70 * [1 1 1] + 0.30 * group_colors(g, :);
    plot(ax1, time_cells{i}, speed_cells{i}, '-', 'Color', c_light, ...
        'LineWidth', 0.8, 'HandleVisibility', 'off');
end
for g = 1:n_groups
    if group_counts(g) > 0
        plot(ax1, t_all, group_mean_curve(g, :), '-', 'Color', group_colors(g, :), ...
            'LineWidth', 2.2, 'DisplayName', sprintf('%s (n=%d)', group_names{g}, group_counts(g)));
    end
end
xlabel(ax1, 'Timestamp');
ylabel(ax1, 'Speed (m/s)');
title(ax1, 'Profil Kecepatan Data Training (4-Way)');
legend(ax1, 'Location', 'eastoutside');

ax2 = nexttile;
valid = find(group_counts > 0);
b = bar(ax2, 1:numel(valid), group_mean_speed(valid), 0.65);
b.FaceColor = 'flat';
for k = 1:numel(valid)
    b.CData(k, :) = group_colors(valid(k), :);
end
hold(ax2, 'on');
errorbar(ax2, 1:numel(valid), group_mean_speed(valid), group_std_speed(valid), ...
    'k.', 'LineWidth', 1.2, 'CapSize', 8);
grid(ax2, 'on');
box(ax2, 'on');
ylabel(ax2, 'Mean Speed (m/s)');
set(ax2, 'XTick', 1:numel(valid), 'XTickLabel', group_names(valid));
title(ax2, 'Rata-rata Kecepatan per Tipe Manuver');
xtickangle(ax2, 15);

print(fig_spd, fullfile(FIG_DIR, 'training_speed_4way.png'), '-dpng', '-r220');
print(fig_spd, fullfile(FIG_DIR, 'training_speed_4way.pdf'), '-dpdf', '-bestfit');
close(fig_spd);

summary_csv = fullfile(FIG_DIR, 'training_speed_4way_summary.csv');
fid = fopen(summary_csv, 'w');
if fid >= 0
    fprintf(fid, 'group,count,mean_speed_mps,std_speed_mps\n');
    for g = 1:n_groups
        if group_counts(g) > 0
            fprintf(fid, '%s,%d,%.6f,%.6f\n', group_names{g}, group_counts(g), ...
                group_mean_speed(g), group_std_speed(g));
        end
    end
    fclose(fid);
end

fprintf('Selesai plot data training 4-way.\n');
fprintf('Output posisi:\n- %s\n- %s\n', ...
    fullfile(FIG_DIR, 'training_position_4way.png'), fullfile(FIG_DIR, 'training_position_4way.pdf'));
fprintf('Output kecepatan:\n- %s\n- %s\n', ...
    fullfile(FIG_DIR, 'training_speed_4way.png'), fullfile(FIG_DIR, 'training_speed_4way.pdf'));
fprintf('Ringkasan kecepatan:\n- %s\n', summary_csv);


function gid = classify_group(sid)
    s = lower(char(sid));
    if contains(s, 'straight_redlight')
        gid = 5;
    elseif contains(s, 'straight_cautious')
        gid = 4;
    elseif contains(s, 'straight')
        gid = 3;
    elseif contains(s, 'left')
        gid = 1;
    elseif contains(s, 'right')
        gid = 2;
    else
        gid = 6;
    end
end
