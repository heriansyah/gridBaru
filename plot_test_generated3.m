clear;
clc;
close all;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'test_generated3.csv');

if ~exist(CSV_FILE, 'file')
    error('File not found: %s', CSV_FILE);
end

fid = fopen(CSV_FILE, 'r');
if fid < 0
    error('Cannot open file: %s', CSV_FILE);
end

tracks = {};
cur_name = "";
cur_xy = zeros(0,2);

while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end

    parts = strsplit(strtrim(line), ',');
    if numel(parts) < 3
        continue;
    end

    scenario = string(parts{1});
    x = str2double(parts{2});
    y = str2double(parts{3});

    if strcmpi(scenario, "SPLIT") || isnan(x) || isnan(y)
        if ~isempty(cur_xy)
            tracks{end+1} = struct('name', cur_name, 'xy', cur_xy); %#ok<AGROW>
            cur_name = "";
            cur_xy = zeros(0,2);
        end
        continue;
    end

    if cur_name == ""
        cur_name = scenario;
    end
    cur_xy(end+1,:) = [x y]; %#ok<AGROW>
end
fclose(fid);

if ~isempty(cur_xy)
    tracks{end+1} = struct('name', cur_name, 'xy', cur_xy); %#ok<AGROW>
end

if isempty(tracks)
    error('No segments found in %s', CSV_FILE);
end

figure('Name', 'test_generated3.csv', 'Color', 'w', ...
       'Position', [150 100 1000 760]);
ax = axes;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
title(ax, 'test\_generated3.csv');

cmap = lines(numel(tracks));
for i = 1:numel(tracks)
    xy = tracks{i}.xy;
    plot(ax, xy(:,1), xy(:,2), '-', 'Color', cmap(i,:), 'LineWidth', 2.0, ...
        'DisplayName', char(tracks{i}.name));
    plot(ax, xy(1,1), xy(1,2), 'o', 'Color', cmap(i,:), ...
        'MarkerFaceColor', cmap(i,:), 'HandleVisibility', 'off');
    plot(ax, xy(end,1), xy(end,2), 's', 'Color', cmap(i,:), ...
        'MarkerFaceColor', cmap(i,:), 'HandleVisibility', 'off');
end

legend(ax, 'Location', 'best');
fprintf('Plotted %d test segment(s) from %s\n', numel(tracks), CSV_FILE);
