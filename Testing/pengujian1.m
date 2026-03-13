% ------------------------------------------------------------
% pengujian1.m (4-way, plot-only dari workspace)
%
% Cara pakai:
% 1) Jalankan dulu: eval_intersection_turns_4way3
% 2) Lalu jalankan:  pengujian1
%
% Script ini TIDAK menghitung evaluasi ulang.
% Script hanya membaca variabel workspace hasil eval:
% - conf, total, correct, skipped
% - opsional: db_hits, no_db_used, labels
% ------------------------------------------------------------

if ~exist('conf', 'var') || ~exist('total', 'var') || ~exist('correct', 'var') || ~exist('skipped', 'var')
    error(['Workspace belum berisi hasil eval 4-way. ' ...
           'Jalankan dulu "eval_intersection_turns_4way3.m", lalu jalankan "pengujian1.m".']);
end

if ~exist('labels', 'var') || numel(labels) ~= 3
    labels = {'left', 'straight', 'right'};
end

if ~exist('db_hits', 'var')
    db_hits = NaN;
end
if ~exist('no_db_used', 'var')
    no_db_used = NaN;
end

acc = 100 * correct / max(total, 1);
wrong = max(total - correct, 0);

% Normalized confusion matrix per-row (actual class).
conf_norm = zeros(size(conf));
for i = 1:size(conf, 1)
    rsum = sum(conf(i, :));
    if rsum > 0
        conf_norm(i, :) = conf(i, :) / rsum;
    end
end

fig = figure('Color', 'w', 'Position', [90 90 1200 460], ...
    'Name', 'Pengujian 1 - 4Way (Plot from Workspace)', 'NumberTitle', 'off');
tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
colormap(fig, 'parula');

% Panel 1: confusion matrix (count)
ax1 = nexttile;
imagesc(ax1, conf);
axis(ax1, 'equal');
axis(ax1, 'tight');
set(ax1, 'XTick', 1:3, 'XTickLabel', labels, 'YTick', 1:3, 'YTickLabel', labels);
xlabel(ax1, 'Predicted');
ylabel(ax1, 'Actual');
title(ax1, 'Confusion Matrix (Count)');
for r = 1:3
    for c = 1:3
        text(ax1, c, r, sprintf('%d', conf(r, c)), ...
            'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold');
    end
end

% Panel 2: confusion matrix (normalized)
ax2 = nexttile;
imagesc(ax2, conf_norm * 100);
axis(ax2, 'equal');
axis(ax2, 'tight');
set(ax2, 'XTick', 1:3, 'XTickLabel', labels, 'YTick', 1:3, 'YTickLabel', labels);
xlabel(ax2, 'Predicted');
ylabel(ax2, 'Actual');
title(ax2, 'Confusion Matrix (%)');
for r = 1:3
    for c = 1:3
        text(ax2, c, r, sprintf('%.1f%%', conf_norm(r, c) * 100), ...
            'HorizontalAlignment', 'center', 'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');
    end
end

% Panel 3: summary bar
ax3 = nexttile;
vals = [correct, wrong, skipped];
b = bar(ax3, vals, 0.6);
b.FaceColor = 'flat';
b.CData = [0.20 0.60 0.25; 0.88 0.38 0.16; 0.45 0.45 0.45];
set(ax3, 'XTick', 1:3, 'XTickLabel', {'Correct', 'Wrong', 'Skipped'});
ylabel(ax3, 'Jumlah Segmen');
title(ax3, sprintf('Summary | Acc = %.2f%%', acc));
grid(ax3, 'on');
box(ax3, 'on');
for k = 1:numel(vals)
    text(ax3, k, vals(k) + 0.5, num2str(vals(k)), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

txt = sprintf('Total: %d\nCorrect: %d\nWrong: %d\nSkipped: %d', total, correct, wrong, skipped);
if ~isnan(db_hits) || ~isnan(no_db_used)
    txt = sprintf('%s\nDB hits: %d\nHeuristic-only: %d', txt, round(db_hits), round(no_db_used));
end
text(ax3, 0.03, 0.97, txt, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'BackgroundColor', [1 1 1], 'Margin', 6, 'EdgeColor', [0.7 0.7 0.7]);

% Simpan ke figs jika folder ada.
SCRIPT_DIR = pwd;
FIG_DIR = fullfile(SCRIPT_DIR, 'figs');
if exist(FIG_DIR, 'dir') ~= 7
    mkdir(FIG_DIR);
end

png_out = fullfile(FIG_DIR, 'pengujian1_4way_workspace.png');
pdf_out = fullfile(FIG_DIR, 'pengujian1_4way_workspace.pdf');
print(fig, png_out, '-dpng', '-r220');
print(fig, pdf_out, '-dpdf', '-bestfit');

fprintf('pengujian1 plot saved:\n- %s\n- %s\n', png_out, pdf_out);
