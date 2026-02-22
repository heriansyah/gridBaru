clear;
clc;
close all;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
SHOW_TRAIN = true;
SHOW_TEST = true;

if ~exist(CSV_FILE, 'file')
    error('File not found: %s', CSV_FILE);
end

T = readtable(CSV_FILE, 'TextType', 'string');
if ~all(ismember(["scenario","timestamp","x","y"], string(T.Properties.VariableNames)))
    error('CSV columns must include: scenario,timestamp,x,y');
end

scn = T.scenario;
x = T.x;
y = T.y;

u = unique(scn, 'stable');
is_test = startsWith(u, "Test_");

figure('Name', 'Custom Trajectories SmartGrid3', 'Color', 'w', ...
       'Position', [120 90 1100 780]);
ax = axes;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
title(ax, 'custom\_trajectories3.csv');

if SHOW_TRAIN
    train_ids = find(~is_test);
    for i = 1:numel(train_ids)
        k = train_ids(i);
        m = (scn == u(k));
        plot(ax, x(m), y(m), '-', 'Color', [0.85 0.15 0.15], 'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end
end

if SHOW_TEST
    test_ids = find(is_test);
    for i = 1:numel(test_ids)
        k = test_ids(i);
        m = (scn == u(k));
        plot(ax, x(m), y(m), '-', 'Color', [1.0 0.0 0.0], 'LineWidth', 2.0, ...
            'DisplayName', char(u(k)));
        plot(ax, x(find(m,1,'first')), y(find(m,1,'first')), 'o', ...
            'Color', [1.0 0.0 0.0], 'MarkerFaceColor', [1.0 0.0 0.0], 'HandleVisibility', 'off');
    end
end

if SHOW_TEST && any(is_test)
    legend(ax, 'Location', 'best');
end

fprintf('Plotted %d scenarios (%d train, %d test)\n', numel(u), sum(~is_test), sum(is_test));
