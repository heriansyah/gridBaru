clear;
clc;
close all;

% ------------------------------------------------------------
% Pengujian khusus skenario lampu merah (4-way, redlight only).
% - Ambil track yang nama skenarionya mengandung "redlight"
% - Evaluasi prediksi manuver (left/straight/right)
% - Simpan plot ringkasan + animasi pergerakan ke folder figs
% ------------------------------------------------------------

SCRIPT_DIR = fileparts(mfilename('fullpath'));
SOURCE_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');  % Sumber data track 4-way
DB_FOLDER = fullfile(SCRIPT_DIR, 'DB_Intersection4way3');         % DB probabilitas p_base per lane
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');                           % Folder output figure

FILTER_KEYWORD = 'redlight';  % Hanya skenario dengan kata ini yang dievaluasi

GRID_SIZE = 10.0;             % Ukuran grid (meter), harus konsisten dengan train/generator
INTERSECTION_RADIUS_M = 25;   % Radius fallback pencarian gid di sekitar pusat (meter)
PRE_POST_WINDOW = 30;         % Titik sebelum/sesudah pusat untuk estimasi heading/turn
TURN_ANGLE_THRESH_DEG = 35;   % Batas sudut klasifikasi straight vs turn
MIN_POINTS = 8;               % Minimal jumlah titik per track agar bisa dievaluasi
INERTIA_ALPHA = 0.5;          % Bobot bias perilaku pada fusi (0=DB-only, 1=behavior-only)
LANE_SHIFT_REF_M = 1.2;       % Referensi drift lateral (meter) untuk normalisasi w_drift
SLOW_GAIN = 0.25;             % Penguatan fitur perlambatan (w_slow)
DRIFT_GAIN = 0.9;             % Penguatan fitur lateral drift (w_drift)
TURN_COMMIT_GAIN = 0.85;      % Seberapa kuat drift menurunkan bias straight (0..1)
ALLOW_NO_DB = true;           % Izinkan fallback heuristic-only saat gid tidak ada di DB
FORCE_NO_DB = false;          % Paksa abaikan DB (uji baseline tanpa p_base)
NO_DB_BASE = [0.25, 0.5, 0.25]; % Prior [left, straight, right] untuk mode no-DB

PLAY_ANIMATION_FIGURE = true; % Tampilkan animasi langsung di figure MATLAB
SAVE_ANIMATION_GIF = false;   % Simpan animasi ke GIF
ANIM_DELAY_SEC = 0.06;       % Delay antar frame GIF (detik)
ANIM_TRAIL_LEN = 14;         % Panjang jejak titik kendaraan (jumlah sampel)
ANIM_HOLD_FRAMES = 5;        % Frame diam di akhir tiap track sebelum lanjut track berikutnya
PRED_MARKER_OFFSET_M = 0.0;  % 0 = tidak digeser (strict). >0 jika ingin geser marker pred saat overlap

if exist(SOURCE_FILE, 'file') ~= 2
    error('Source file not found: %s', SOURCE_FILE);
end

if exist(FIG_DIR, 'dir') ~= 7
    mkdir(FIG_DIR);
end

use_db = exist(DB_FOLDER, 'dir') == 7 && ~FORCE_NO_DB;
if ~use_db && ~ALLOW_NO_DB
    error('DB folder not found: %s', DB_FOLDER);
end

fprintf('=== PENGUJIAN LAMPU MERAH 4-WAY (REDLIGHT ONLY) ===\n');
fprintf('Source file: %s\n', SOURCE_FILE);
fprintf('Filter scenario: *%s*\n', FILTER_KEYWORD);
if use_db
    fprintf('DB mode: ON (%s)\n', DB_FOLDER);
else
    fprintf('DB mode: OFF (heuristic-only)\n');
end

tracks_all = load_tracks_by_scenario(SOURCE_FILE);
mask_red = false(numel(tracks_all), 1);
for i = 1:numel(tracks_all)
    s = lower(tracks_all{i}.scenario);
    mask_red(i) = contains(s, lower(FILTER_KEYWORD));
end
tracks = tracks_all(mask_red);

if isempty(tracks)
    error(['Tidak ada skenario redlight ditemukan. ' ...
           'Pastikan data dibuat dengan generate_custom_data3.m dan memiliki nama "*straight_redlight*".']);
end

train_red = 0;
test_red = 0;
for i = 1:numel(tracks)
    s = string(tracks{i}.scenario);
    if startsWith(s, "Train_")
        train_red = train_red + 1;
    elseif startsWith(s, "Test_")
        test_red = test_red + 1;
    end
end
fprintf('Redlight split        : train=%d, test=%d\n', train_red, test_red);
if test_red == 0 && train_red > 0
    fprintf(['WARNING: Semua redlight track berasal dari TRAIN set. ' ...
             'Akurasi bisa terlalu optimistis (data leakage).\n']);
end

if use_db
    db = load_database(DB_FOLDER);
else
    db = cell(4, 1);
end

labels = {'left', 'straight', 'right'};
conf = zeros(3, 3);
total = 0;
correct = 0;
skipped = 0;
db_hits = 0;
no_db_used = 0;

speed_ratio_all = [];
lat_delta_all = [];
pred_idx_all = [];
actual_idx_all = [];
eval_records = struct( ...
    'scenario', {}, 't', {}, 'x', {}, 'y', {}, 'idx0', {}, ...
    'lane', {}, ...
    'actual_idx', {}, 'pred_idx', {}, 'is_correct', {}, ...
    'speed_t', {}, 'speed', {}, 'speed_ratio', {}, 'lat_delta', {});

for s = 1:numel(tracks)
    xy = tracks{s}.xy;
    if size(xy, 1) < MIN_POINTS
        skipped = skipped + 1;
        continue;
    end

    x = xy(:, 1);
    y = xy(:, 2);
    d = hypot(x, y);
    [~, idx0] = min(d);

    i1 = max(1, idx0 - PRE_POST_WINDOW);
    i2 = min(numel(x), idx0 + PRE_POST_WINDOW);

    v_in = mean_vec(x, y, i1, idx0);
    v_out = mean_vec(x, y, idx0, i2);
    if any(isnan(v_in)) || any(isnan(v_out))
        skipped = skipped + 1;
        continue;
    end

    heading = mod(atan2(v_in(1), v_in(2)) * 180 / pi, 360);
    lane = lane_from_heading(heading);

    actual = turn_from_label_then_vector(tracks{s}.scenario, v_in, v_out, TURN_ANGLE_THRESH_DEG);
    if isempty(actual)
        skipped = skipped + 1;
        continue;
    end

    gx = floor(x(idx0) / GRID_SIZE) + 1;
    gy = floor(y(idx0) / GRID_SIZE) + 1;
    gid = int32((gy - 1) * 2000 + (gx - 1));

    pred = '';
    if use_db
        pred = predict_turn_with_inertia( ...
            db{lane}, gid, lane, x, y, idx0, v_in, ...
            INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN, TURN_COMMIT_GAIN);
        if isempty(pred)
            in_mask = d <= INTERSECTION_RADIUS_M;
            if any(in_mask)
                gx2 = floor(x(in_mask) / GRID_SIZE) + 1;
                gy2 = floor(y(in_mask) / GRID_SIZE) + 1;
                gids = int32((gy2 - 1) * 2000 + (gx2 - 1));
                for k = 1:numel(gids)
                    pred = predict_turn_with_inertia( ...
                        db{lane}, gids(k), lane, x, y, idx0, v_in, ...
                        INERTIA_ALPHA, LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN, TURN_COMMIT_GAIN);
                    if ~isempty(pred)
                        break;
                    end
                end
            end
        end
        if ~isempty(pred)
            db_hits = db_hits + 1;
        end
    end

    if isempty(pred) && ALLOW_NO_DB
        pred = predict_turn_no_db( ...
            x, y, idx0, v_in, NO_DB_BASE, INERTIA_ALPHA, ...
            LANE_SHIFT_REF_M, SLOW_GAIN, DRIFT_GAIN, TURN_COMMIT_GAIN);
        if ~isempty(pred)
            no_db_used = no_db_used + 1;
        end
    end

    if isempty(pred)
        skipped = skipped + 1;
        continue;
    end

    [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in);

    ai = turn_to_index(actual);
    pi = turn_to_index(pred);
    conf(ai, pi) = conf(ai, pi) + 1;
    total = total + 1;
    if ai == pi
        correct = correct + 1;
    end

    speed_ratio_all(end + 1, 1) = speed_ratio; %#ok<AGROW>
    lat_delta_all(end + 1, 1) = lat_delta; %#ok<AGROW>
    pred_idx_all(end + 1, 1) = pi; %#ok<AGROW>
    actual_idx_all(end + 1, 1) = ai; %#ok<AGROW>

    tt = tracks{s}.t(:);
    dt = diff(tt);
    ds = hypot(diff(x), diff(y));
    v_profile = ds ./ max(dt, 1e-6);
    t_profile = tt(2:end);

    rec = struct();
    rec.scenario = tracks{s}.scenario;
    rec.t = tt;
    rec.x = x;
    rec.y = y;
    rec.idx0 = idx0;
    rec.lane = lane;
    rec.actual_idx = ai;
    rec.pred_idx = pi;
    rec.is_correct = (ai == pi);
    rec.speed_t = t_profile;
    rec.speed = v_profile;
    rec.speed_ratio = speed_ratio;
    rec.lat_delta = lat_delta;
    eval_records(end + 1) = rec; %#ok<AGROW>

    fprintf('Seg %2d | %-40s | actual=%s | pred=%s | speedRatio=%.3f\n', ...
        s, tracks{s}.scenario, actual, pred, speed_ratio);
end

stats = confusion_stats(conf);
total_from_conf = stats.total;
correct_from_conf = stats.correct;
wrong_from_conf = stats.wrong;
acc = stats.accuracy;
per_class_total = stats.support;
per_class_correct = stats.tp;
per_class_acc = stats.recall * 100;

if total ~= total_from_conf || correct ~= correct_from_conf
    fprintf(['WARNING: counter manual tidak sama dengan confusion matrix. ' ...
             'Statistik akhir menggunakan confusion matrix.\n']);
end

fprintf('\n=== SUMMARY REDLIGHT ===\n');
fprintf('Tracks redlight found : %d\n', numel(tracks));
fprintf('Total evaluated       : %d\n', total_from_conf);
fprintf('Correct               : %d\n', correct_from_conf);
fprintf('Accuracy              : %.2f%%\n', acc);
fprintf('Skipped               : %d\n', skipped);
fprintf('DB hits               : %d\n', db_hits);
fprintf('Heuristic-only        : %d\n', no_db_used);
fprintf('Per-class accuracy    : left=%.2f%%, straight=%.2f%%, right=%.2f%%\n', ...
    per_class_acc(1), per_class_acc(2), per_class_acc(3));
fprintf('Class coverage        : left=%d, straight=%d, right=%d\n', ...
    per_class_total(1), per_class_total(2), per_class_total(3));
fprintf('Per-class precision   : left=%.2f%%, straight=%.2f%%, right=%.2f%%\n', ...
    stats.precision(1) * 100, stats.precision(2) * 100, stats.precision(3) * 100);
fprintf('Per-class F1-score    : left=%.2f%%, straight=%.2f%%, right=%.2f%%\n', ...
    stats.f1(1) * 100, stats.f1(2) * 100, stats.f1(3) * 100);
fprintf('Macro-F1              : %.2f%%\n', stats.macro_f1 * 100);
fprintf('Balanced Accuracy     : %.2f%%\n', stats.balanced_acc * 100);
fprintf('Mismatch count        : %d\n', stats.wrong);

% ------------------------------
% Plot 1: confusion + summary
% ------------------------------
fig1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 380]);
tiledlayout(fig1, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
colormap(fig1, 'parula');

ax1 = nexttile;
imagesc(ax1, conf);
axis(ax1, 'equal');
axis(ax1, 'tight');
set(ax1, 'XTick', 1:3, 'XTickLabel', labels, 'YTick', 1:3, 'YTickLabel', labels);
xlabel(ax1, 'Predicted');
ylabel(ax1, 'Actual');
title(ax1, 'Confusion Matrix (Redlight Only)');
for r = 1:3
    for c = 1:3
        text(ax1, c, r, sprintf('%d', conf(r, c)), ...
            'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold');
    end
end

ax2 = nexttile;
vals = [correct_from_conf, wrong_from_conf, skipped];
b = bar(ax2, vals, 0.58);
b.FaceColor = 'flat';
b.CData = [0.22 0.62 0.27; 0.87 0.35 0.18; 0.45 0.45 0.45];
set(ax2, 'XTick', 1:3, 'XTickLabel', {'Correct', 'Wrong', 'Skipped'});
ylabel(ax2, 'Count');
title(ax2, sprintf('Summary | Acc = %.2f%%', acc));
grid(ax2, 'on');
box(ax2, 'on');
for k = 1:numel(vals)
    text(ax2, k, vals(k) + 0.2, sprintf('%d', vals(k)), 'HorizontalAlignment', 'center');
end

cm_png = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_cm.png');
cm_pdf = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_cm.pdf');
print(fig1, cm_png, '-dpng', '-r220');
print(fig1, cm_pdf, '-dpdf', '-bestfit');
close(fig1);

% ------------------------------
% Plot 2: cue scatter
% ------------------------------
fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 760 420]);
ax = axes(fig2);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

if ~isempty(speed_ratio_all)
    c = zeros(numel(pred_idx_all), 3);
    for i = 1:numel(pred_idx_all)
        if pred_idx_all(i) == 1
            c(i, :) = [0.20 0.50 0.85];
        elseif pred_idx_all(i) == 2
            c(i, :) = [0.24 0.62 0.24];
        else
            c(i, :) = [0.86 0.35 0.16];
        end
    end
    scatter(ax, speed_ratio_all, lat_delta_all, 42, c, 'filled');
end

xlabel(ax, 'Speed Ratio (center / before)');
ylabel(ax, 'Lateral Drift \Delta_{lat} (m)');
title(ax, 'Behavior Cues on Redlight Tracks');
plot(ax, [1 1], ylim(ax), '--k', 'LineWidth', 1.0, 'HandleVisibility', 'off');

plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.20 0.50 0.85], 'MarkerEdgeColor', [0.20 0.50 0.85], 'DisplayName', 'pred=left');
plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.24 0.62 0.24], 'MarkerEdgeColor', [0.24 0.62 0.24], 'DisplayName', 'pred=straight');
plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.86 0.35 0.16], 'MarkerEdgeColor', [0.86 0.35 0.16], 'DisplayName', 'pred=right');
legend(ax, 'Location', 'best');

cue_png = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_cues.png');
cue_pdf = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_cues.pdf');
print(fig2, cue_png, '-dpng', '-r220');
print(fig2, cue_pdf, '-dpdf', '-bestfit');
close(fig2);

% ------------------------------
% Plot 3: accuracy per maneuver redlight
% ------------------------------
fig3 = figure('Visible', 'off', 'Color', 'w', 'Position', [140 140 700 420]);
yyaxis left;
b = bar(1:3, per_class_acc, 0.6);
b.FaceColor = 'flat';
b.CData = [0.20 0.50 0.85; 0.24 0.62 0.24; 0.86 0.35 0.16];
ylim([0 100]);
ylabel('Accuracy (%)');
set(gca, 'XTick', 1:3, 'XTickLabel', labels);
grid on;
box on;

yyaxis right;
plot(1:3, per_class_total, 'k-o', 'LineWidth', 1.4, 'MarkerFaceColor', 'k');
ylabel('Jumlah Sampel');

title('Redlight Accuracy per Maneuver (Left/Straight/Right)');
for i = 1:3
    yyaxis left;
    text(i, per_class_acc(i) + 2, sprintf('%.1f%%', per_class_acc(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end

maneuver_png = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_per_maneuver.png');
maneuver_pdf = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_per_maneuver.pdf');
print(fig3, maneuver_png, '-dpng', '-r220');
print(fig3, maneuver_pdf, '-dpdf', '-bestfit');
close(fig3);

% ------------------------------
% Plot 4: trajectory overview (redlight tracks)
% ------------------------------
fig4 = figure('Visible', 'off', 'Color', 'w', 'Position', [90 80 980 680]);
ax4 = axes(fig4);
hold(ax4, 'on');
grid(ax4, 'on');
box(ax4, 'on');
axis(ax4, 'equal');
xlabel(ax4, 'X (m)');
ylabel(ax4, 'Y (m)');
title(ax4, 'Trajectory Redlight (Warna=Prediksi, Style=Benar/Salah)');

for i = 1:numel(eval_records)
    rr = eval_records(i);
    c = class_color(rr.pred_idx);
    lw = 1.9;
    ls = '-';
    if ~rr.is_correct
        ls = '--';
        lw = 2.2;
    end

    plot(ax4, rr.x, rr.y, 'LineStyle', ls, 'Color', c, 'LineWidth', lw, 'HandleVisibility', 'off');
    plot(ax4, rr.x(rr.idx0), rr.y(rr.idx0), 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    text(rr.x(rr.idx0), rr.y(rr.idx0), sprintf(' %d', i), 'FontSize', 8, 'Color', [0.15 0.15 0.15]);
end

plot(ax4, 0, 0, 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12, 'DisplayName', 'Pusat Persimpangan');
plot(ax4, NaN, NaN, '-', 'Color', class_color(1), 'LineWidth', 2.0, 'DisplayName', 'pred=left');
plot(ax4, NaN, NaN, '-', 'Color', class_color(2), 'LineWidth', 2.0, 'DisplayName', 'pred=straight');
plot(ax4, NaN, NaN, '-', 'Color', class_color(3), 'LineWidth', 2.0, 'DisplayName', 'pred=right');
plot(ax4, NaN, NaN, 'k-', 'LineWidth', 2.0, 'DisplayName', 'benar');
plot(ax4, NaN, NaN, 'k--', 'LineWidth', 2.0, 'DisplayName', 'salah');
legend(ax4, 'Location', 'eastoutside');

traj_png = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_trajectories.png');
traj_pdf = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_trajectories.pdf');
print(fig4, traj_png, '-dpng', '-r220');
print(fig4, traj_pdf, '-dpdf', '-bestfit');
close(fig4);

% ------------------------------
% Plot 5: speed profile per redlight track
% ------------------------------
fig5 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 90 980 470]);
ax5 = axes(fig5);
hold(ax5, 'on');
grid(ax5, 'on');
box(ax5, 'on');
xlabel(ax5, 'Timestamp');
ylabel(ax5, 'Speed (m/s)');
title(ax5, 'Speed Profile Tiap Track Redlight');

max_t = 0;
for i = 1:numel(eval_records)
    rr = eval_records(i);
    c = class_color(rr.actual_idx);
    plot(ax5, rr.speed_t, rr.speed, '-', 'Color', c, 'LineWidth', 1.4, 'HandleVisibility', 'off');
    if ~isempty(rr.speed_t)
        max_t = max(max_t, max(rr.speed_t));
    end
end

if max_t > 0
    xlim(ax5, [0 max_t]);
end

plot(ax5, NaN, NaN, '-', 'Color', class_color(1), 'LineWidth', 2.0, 'DisplayName', 'actual=left');
plot(ax5, NaN, NaN, '-', 'Color', class_color(2), 'LineWidth', 2.0, 'DisplayName', 'actual=straight');
plot(ax5, NaN, NaN, '-', 'Color', class_color(3), 'LineWidth', 2.0, 'DisplayName', 'actual=right');
legend(ax5, 'Location', 'eastoutside');

spd_png = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_speed_profiles.png');
spd_pdf = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_speed_profiles.pdf');
print(fig5, spd_png, '-dpng', '-r220');
print(fig5, spd_pdf, '-dpdf', '-bestfit');
close(fig5);

% ------------------------------
% Plot 6: animasi pergerakan redlight (GIF)
% ------------------------------
anim_gif = fullfile(FIG_DIR, 'pengujian_lampumerah_4way_redlight_animation.gif');
if (PLAY_ANIMATION_FIGURE || SAVE_ANIMATION_GIF) && ~isempty(eval_records)
    topo = topology_4way3();
    fig6_vis = 'off';
    if PLAY_ANIMATION_FIGURE
        fig6_vis = 'on';
    end
    fig6 = figure('Visible', fig6_vis, 'Color', 'w', 'Position', [120 90 1100 700]);
    ax6 = axes(fig6);
    hold(ax6, 'on');
    axis(ax6, 'equal');
    grid(ax6, 'on');
    box(ax6, 'on');
    xlabel(ax6, 'X (m)');
    ylabel(ax6, 'Y (m)');
    xlim(ax6, topo.bounds(1:2));
    ylim(ax6, topo.bounds(3:4));

    car_length = 4.4;
    car_width = 1.8;
    car_shape_local = [
        -car_width/2, -car_length/2;
         car_width/2, -car_length/2;
         car_width/2,  car_length/2;
        -car_width/2,  car_length/2
    ];

    first_frame = true;
    tmp_frame_png = fullfile(FIG_DIR, '__tmp_redlight_anim_frame.png');

    for i = 1:numel(eval_records)
        rr = eval_records(i);
        cla(ax6);
        hold(ax6, 'on');
        axis(ax6, 'equal');
        grid(ax6, 'on');
        box(ax6, 'on');
        xlabel(ax6, 'X (m)');
        ylabel(ax6, 'Y (m)');
        xlim(ax6, topo.bounds(1:2));
        ylim(ax6, topo.bounds(3:4));

        h_road = plot(topo.road_poly, 'FaceColor', [0.92 0.92 0.92], ...
            'EdgeColor', [0.25 0.25 0.25], 'DisplayName', 'Road');
        h_lane = gobjects(1);
        for kk = 1:numel(topo.lane_markings)
            lm = topo.lane_markings{kk};
            if kk == 1
                h_lane = plot(ax6, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], ...
                    'LineWidth', 1.0, 'DisplayName', 'Lane Marking');
            else
                plot(ax6, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], ...
                    'LineWidth', 1.0, 'HandleVisibility', 'off');
            end
        end

        h_path = plot(ax6, rr.x, rr.y, '-', 'Color', [0.45 0.45 0.45], ...
            'LineWidth', 1.1, 'DisplayName', 'Full Path');
        h_start = plot(ax6, rr.x(1), rr.y(1), 'ko', 'MarkerFaceColor', 'w', ...
            'MarkerSize', 6, 'DisplayName', 'Start');
        center_xy = [rr.x(rr.idx0), rr.y(rr.idx0)];
        h_center = plot(ax6, center_xy(1), center_xy(2), 'kp', 'MarkerFaceColor', 'y', ...
            'MarkerSize', 9, 'DisplayName', 'Center');

        u_pred = turn_to_vector(rr.lane, labels{rr.pred_idx});
        u_act = turn_to_vector(rr.lane, labels{rr.actual_idx});
        pred_xy = center_xy + u_pred * 35;
        act_xy = center_xy + u_act * 35;

        % Opsional: geser marker prediksi jika overlap total.
        if PRED_MARKER_OFFSET_M > 0 && norm(pred_xy - act_xy) < 0.25
            pred_xy = pred_xy + [-u_act(2), u_act(1)] * PRED_MARKER_OFFSET_M;
        end

        h_actual = plot(ax6, act_xy(1), act_xy(2), 'gs', 'MarkerFaceColor', 'g', ...
            'MarkerSize', 8, 'DisplayName', 'Actual');
        h_pred = plot(ax6, pred_xy(1), pred_xy(2), 'rx', 'LineWidth', 2.8, ...
            'MarkerSize', 13, 'DisplayName', 'Predicted');
        text(ax6, act_xy(1) + 0.8, act_xy(2) + 0.8, 'A', 'Color', [0.0 0.45 0.0], ...
            'FontWeight', 'bold', 'FontSize', 9, 'HandleVisibility', 'off');
        text(ax6, pred_xy(1) + 0.8, pred_xy(2) + 0.8, 'P', 'Color', [0.75 0.0 0.0], ...
            'FontWeight', 'bold', 'FontSize', 9, 'HandleVisibility', 'off');

        c = class_color(rr.pred_idx);
        h_trail = plot(ax6, rr.x(1), rr.y(1), '-', 'Color', c, 'LineWidth', 2.0, 'DisplayName', 'Trail');
        h_car = patch(ax6, NaN, NaN, c, 'FaceAlpha', 0.9, 'EdgeColor', [0 0 0], ...
            'LineWidth', 1.0, 'DisplayName', 'Vehicle');
        h_head = quiver(ax6, NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 2.0, ...
            'MaxHeadSize', 2.0, 'DisplayName', 'Heading');

        lgd = legend(ax6, [h_road, h_lane, h_path, h_start, h_center, h_pred, h_actual, h_trail, h_car, h_head], ...
            {'Road', 'Lane Marking', 'Full Path', 'Start', 'Center', 'Predicted', 'Actual', 'Trail', 'Vehicle', 'Heading'}, ...
            'Location', 'eastoutside');
        set(lgd, 'AutoUpdate', 'off');

        if rr.is_correct
            verdict = 'CORRECT';
        else
            verdict = 'WRONG';
        end

        for k = 1:numel(rr.x)
            i1 = max(1, k - ANIM_TRAIL_LEN + 1);
            set(h_trail, 'XData', rr.x(i1:k), 'YData', rr.y(i1:k));

            if k < numel(rr.x)
                dx = rr.x(k+1) - rr.x(k);
                dy = rr.y(k+1) - rr.y(k);
            else
                dx = rr.x(k) - rr.x(max(k-1, 1));
                dy = rr.y(k) - rr.y(max(k-1, 1));
            end
            theta = atan2(dy, dx);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            car_world = (R * car_shape_local')';
            car_world(:,1) = car_world(:,1) + rr.x(k);
            car_world(:,2) = car_world(:,2) + rr.y(k);
            set(h_car, 'XData', car_world(:,1), 'YData', car_world(:,2));

            dir_norm = max(hypot(dx, dy), 1e-6);
            ux = dx / dir_norm;
            uy = dy / dir_norm;
            set(h_head, 'XData', rr.x(k), 'YData', rr.y(k), 'UData', ux * 6, 'VData', uy * 6);

            title(ax6, sprintf('Redlight Track %d/%d | %s | actual=%s pred=%s | %s', ...
                i, numel(eval_records), rr.scenario, labels{rr.actual_idx}, labels{rr.pred_idx}, verdict), ...
                'Interpreter', 'none', 'FontWeight', 'bold');

            drawnow;
            if PLAY_ANIMATION_FIGURE
                pause(ANIM_DELAY_SEC);
            end
            if SAVE_ANIMATION_GIF
                append_gif_frame(fig6, anim_gif, first_frame, ANIM_DELAY_SEC, tmp_frame_png);
                if first_frame
                    first_frame = false;
                end
            end
        end

        for h = 1:ANIM_HOLD_FRAMES
            if PLAY_ANIMATION_FIGURE
                pause(ANIM_DELAY_SEC);
            end
            if SAVE_ANIMATION_GIF
                append_gif_frame(fig6, anim_gif, first_frame, ANIM_DELAY_SEC, tmp_frame_png);
            end
        end
    end

    if SAVE_ANIMATION_GIF && exist(tmp_frame_png, 'file') == 2
        delete(tmp_frame_png);
    end
    if ~PLAY_ANIMATION_FIGURE
        close(fig6);
    end
end

fprintf('\nSaved plots:\n- %s\n- %s\n- %s\n- %s\n- %s\n- %s\n- %s\n- %s\n- %s\n- %s\n', ...
    cm_png, cm_pdf, cue_png, cue_pdf, maneuver_png, maneuver_pdf, traj_png, traj_pdf, spd_png, spd_pdf);
if PLAY_ANIMATION_FIGURE
    fprintf('Animation mode:\n- MATLAB figure playback ON\n');
end
if SAVE_ANIMATION_GIF
    fprintf('Saved animation:\n- %s\n', anim_gif);
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

function t = turn_from_label_then_vector(sid, v_in, v_out, thresh_deg)
    s = lower(sid);
    if contains(s, 'left')
        t = 'left';
        return;
    end
    if contains(s, 'right')
        t = 'right';
        return;
    end
    if contains(s, 'straight')
        t = 'straight';
        return;
    end
    t = turn_from_vectors(v_in, v_out, thresh_deg);
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

function pred = predict_turn_with_inertia(map, gid, lane, x, y, idx0, v_in, alpha, lane_shift_ref, slow_gain, drift_gain, turn_commit_gain)
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

    % Redlight-aware bias:
    % jika drift mulai kuat, komponen straight diturunkan agar niat belok muncul.
    turn_factor = clamp_value(abs(lat_delta) / max(lane_shift_ref, 1e-6), 0, 1);
    p_straight = (1 - slow_w) * (1 - turn_commit_gain * turn_factor);
    p_straight = clamp_value(p_straight, 0, 1);
    p_turn_total = max(1 - p_straight, 0);
    p_bias = [0.5 * p_turn_total, p_straight, 0.5 * p_turn_total];

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

function pred = predict_turn_no_db(x, y, idx0, v_in, base_probs, alpha, lane_shift_ref, slow_gain, drift_gain, turn_commit_gain)
    pred = '';
    if any(isnan(v_in))
        return;
    end
    if numel(base_probs) ~= 3
        base_probs = [1, 1, 1];
    end
    p_base = normalize_probs(base_probs);

    [speed_ratio, lat_delta] = motion_cues(x, y, idx0, v_in);
    slow_w = clamp_value((1 - speed_ratio) * slow_gain, 0, 0.6);
    drift_w = clamp_value((abs(lat_delta) / max(lane_shift_ref, 1e-6)) * drift_gain, 0, 0.6);

    turn_factor = clamp_value(abs(lat_delta) / max(lane_shift_ref, 1e-6), 0, 1);
    p_straight = (1 - slow_w) * (1 - turn_commit_gain * turn_factor);
    p_straight = clamp_value(p_straight, 0, 1);
    p_turn_total = max(1 - p_straight, 0);
    p_bias = [0.5 * p_turn_total, p_straight, 0.5 * p_turn_total];

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

function p = normalize_probs(p)
    s = sum(p);
    if s > 0
        p = p / s;
    end
end

function v = clamp_value(v, vmin, vmax)
    v = max(vmin, min(vmax, v));
end

function [left_idx, straight_idx, right_idx] = lane_turn_indices(lane)
    if lane == 1
        left_idx = 7;
        straight_idx = 1;
        right_idx = 3;
    elseif lane == 2
        left_idx = 1;
        straight_idx = 3;
        right_idx = 5;
    elseif lane == 3
        left_idx = 3;
        straight_idx = 5;
        right_idx = 7;
    else
        left_idx = 5;
        straight_idx = 7;
        right_idx = 1;
    end
end

function c = class_color(idx)
    if idx == 1
        c = [0.20, 0.50, 0.85]; % left
    elseif idx == 2
        c = [0.24, 0.62, 0.24]; % straight
    else
        c = [0.86, 0.35, 0.16]; % right
    end
end

function u = turn_to_vector(lane, turn)
    % Unit vector untuk visual marker prediksi/aktual dari pusat persimpangan.
    switch lane
        case 1 % moving north
            if strcmp(turn, 'left')
                u = [-1, 0];
            elseif strcmp(turn, 'right')
                u = [1, 0];
            else
                u = [0, 1];
            end
        case 2 % moving east
            if strcmp(turn, 'left')
                u = [0, 1];
            elseif strcmp(turn, 'right')
                u = [0, -1];
            else
                u = [1, 0];
            end
        case 3 % moving south
            if strcmp(turn, 'left')
                u = [1, 0];
            elseif strcmp(turn, 'right')
                u = [-1, 0];
            else
                u = [0, -1];
            end
        otherwise % lane 4 moving west
            if strcmp(turn, 'left')
                u = [0, -1];
            elseif strcmp(turn, 'right')
                u = [0, 1];
            else
                u = [-1, 0];
            end
    end
end

function append_gif_frame(fig_handle, gif_file, is_first, delay_sec, tmp_png)
    try
        fr = getframe(fig_handle);
        [im, map] = rgb2ind(frame2im(fr), 256);
    catch
        exportgraphics(fig_handle, tmp_png, 'Resolution', 130);
        rgb = imread(tmp_png);
        [im, map] = rgb2ind(rgb, 256);
    end

    if is_first
        imwrite(im, map, gif_file, 'gif', 'LoopCount', inf, 'DelayTime', delay_sec);
    else
        imwrite(im, map, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', delay_sec);
    end
end

function s = confusion_stats(conf)
    tp = diag(conf);
    support = sum(conf, 2);
    pred_count = sum(conf, 1)';

    total = sum(conf(:));
    correct = sum(tp);
    wrong = max(total - correct, 0);
    acc = correct / max(total, 1);

    precision = safe_div(tp, pred_count);
    recall = safe_div(tp, support);
    f1 = safe_div(2 .* precision .* recall, (precision + recall));

    valid_cls = support > 0;
    if any(valid_cls)
        macro_f1 = mean(f1(valid_cls), 'omitnan');
        balanced_acc = mean(recall(valid_cls), 'omitnan');
    else
        macro_f1 = 0;
        balanced_acc = 0;
    end

    s = struct();
    s.total = total;
    s.correct = correct;
    s.wrong = wrong;
    s.accuracy = 100 * acc;
    s.tp = tp;
    s.support = support;
    s.precision = precision;
    s.recall = recall;
    s.f1 = f1;
    s.macro_f1 = macro_f1;
    s.balanced_acc = balanced_acc;
end

function out = safe_div(num, den)
    out = zeros(size(num));
    mask = den > 0;
    out(mask) = num(mask) ./ den(mask);
    out(~mask) = 0;
end

function tracks = load_tracks_by_scenario(fname)
    fid = fopen(fname, 'r');
    if fid < 0
        error('Could not open source file: %s', fname);
    end
    header = fgetl(fid); %#ok<NASGU>
    data = textscan(fid, '%s %f %f %f', 'Delimiter', ',', 'CollectOutput', false);
    fclose(fid);

    scenarios = data{1};
    ts = data{2};
    xs = data{3};
    ys = data{4};

    valid = ~isnan(xs) & ~isnan(ys) & ~isnan(ts);
    scenarios = scenarios(valid);
    ts = ts(valid);
    xs = xs(valid);
    ys = ys(valid);

    [uniq, ~] = unique(scenarios, 'stable');
    tracks = cell(numel(uniq), 1);
    for i = 1:numel(uniq)
        sid = uniq{i};
        m = strcmp(scenarios, sid);
        t = ts(m);
        x = xs(m);
        y = ys(m);
        [t, ord] = sort(t);
        x = x(ord);
        y = y(ord);
        tracks{i} = struct('scenario', sid, 't', t, 'xy', [x, y]);
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
