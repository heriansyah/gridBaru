clear;
clc;
close all;

% ------------------------------------------------------------
% Plot kecepatan data training 4-way dari custom_trajectories3.csv
% Output:
% - figs/training_speed_profiles_4way.png
% - figs/training_speed_profiles_4way.pdf
% - figs/training_speed_summary_4way.csv
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');

if ~exist(CSV_FILE, 'file')
    error('File not found: %s', CSV_FILE);
end
if ~exist(FIG_DIR, 'dir')
    mkdir(FIG_DIR);
end

T = readtable(CSV_FILE, 'TextType', 'string');
must_cols = ["scenario", "timestamp", "x", "y"];
if ~all(ismember(must_cols, string(T.Properties.VariableNames)))
    error('CSV wajib memiliki kolom: scenario,timestamp,x,y');
end

is_train = startsWith(T.scenario, "Train_");
T = T(is_train, :);
if isempty(T)
    error('Tidak ada baris training (scenario dengan prefix Train_)');
end

scenarios = unique(T.scenario, 'stable');
n_scen = numel(scenarios);

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

speed_cells = cell(n_scen, 1);
time_cells = cell(n_scen, 1);
scenario_group = zeros(n_scen, 1);

for i = 1:n_scen
    sid = scenarios(i);
    m = (T.scenario == sid);
    ts = T.timestamp(m);
    x = T.x(m);
    y = T.y(m);

    [ts, ord] = sort(ts);
    x = x(ord);
    y = y(ord);

    dt = diff(ts);
    ds = hypot(diff(x), diff(y));
    v = ds ./ max(dt, 1e-6);
    tmid = ts(2:end);

    speed_cells{i} = v(:);
    time_cells{i} = tmid(:);
    scenario_group(i) = classify_group(sid);
end

% Bangun grid waktu referensi (asumsi dt seragam; tetap aman jika tidak).
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

group_mean_curve = NaN(n_groups, n_t);
group_mean_speed = NaN(n_groups, 1);
group_std_speed = NaN(n_groups, 1);
group_counts = zeros(n_groups, 1);

for g = 1:n_groups
    idx = (scenario_group == g);
    group_counts(g) = sum(idx);
    if any(idx)
        group_mean_curve(g, :) = mean(Vmat(idx, :), 1, 'omitnan');
        vals = Vmat(idx, :);
        vals = vals(~isnan(vals));
        group_mean_speed(g) = mean(vals);
        group_std_speed(g) = std(vals);
    end
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 70 1100 760]);
tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Panel 1: semua kurva skenario (tipis) + rata-rata per grup (tebal)
ax1 = nexttile;
hold(ax1, 'on');
grid(ax1, 'on');
box(ax1, 'on');

for i = 1:n_scen
    g = scenario_group(i);
    base_c = group_colors(g, :);
    c = 0.70 * [1 1 1] + 0.30 * base_c; % versi lebih terang
    plot(ax1, time_cells{i}, speed_cells{i}, '-', 'Color', c, 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
end

for g = 1:n_groups
    if group_counts(g) > 0
        plot(ax1, t_all, group_mean_curve(g, :), '-', 'Color', group_colors(g, :), ...
            'LineWidth', 2.2, 'DisplayName', sprintf('%s (n=%d)', group_names{g}, group_counts(g)));
    end
end

xlabel(ax1, 'Timestamp');
ylabel(ax1, 'Speed (m/s)');
title(ax1, 'Kecepatan Training per Skenario (4-Way)');
legend(ax1, 'Location', 'eastoutside');

% Panel 2: bar mean speed per grup + errorbar std
ax2 = nexttile;
valid_g = find(group_counts > 0);
b = bar(ax2, 1:numel(valid_g), group_mean_speed(valid_g), 0.65);
b.FaceColor = 'flat';
for k = 1:numel(valid_g)
    b.CData(k, :) = group_colors(valid_g(k), :);
end
hold(ax2, 'on');
errorbar(ax2, 1:numel(valid_g), group_mean_speed(valid_g), group_std_speed(valid_g), ...
    'k.', 'LineWidth', 1.2, 'CapSize', 8);
grid(ax2, 'on');
box(ax2, 'on');
ylabel(ax2, 'Mean Speed (m/s)');
set(ax2, 'XTick', 1:numel(valid_g), 'XTickLabel', group_names(valid_g));
title(ax2, 'Rata-rata Kecepatan per Tipe Manuver');
xtickangle(ax2, 15);

png_out = fullfile(FIG_DIR, 'training_speed_profiles_4way.png');
pdf_out = fullfile(FIG_DIR, 'training_speed_profiles_4way.pdf');
print(fig, png_out, '-dpng', '-r220');
print(fig, pdf_out, '-dpdf', '-bestfit');
close(fig);

summary_file = fullfile(FIG_DIR, 'training_speed_summary_4way.csv');
fid = fopen(summary_file, 'w');
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

fprintf('Plot saved:\n- %s\n- %s\n', png_out, pdf_out);
fprintf('Summary saved:\n- %s\n', summary_file);


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
