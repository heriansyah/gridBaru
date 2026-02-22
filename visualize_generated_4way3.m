clear;
clc;
close all;

SCRIPT_DIR = fileparts(mfilename('fullpath'));
CSV_FILE = fullfile(SCRIPT_DIR, 'custom_trajectories3.csv');
TARGET_SCENARIO = 'Test_4way_Veh_001_left';
PAUSE_SEC = 0.04;

if ~exist(CSV_FILE, 'file')
    error('File not found: %s. Run generate_custom_data3.m first.', CSV_FILE);
end

fid = fopen(CSV_FILE, 'r');
if fid < 0
    error('Cannot open %s', CSV_FILE);
end
header = fgetl(fid); %#ok<NASGU>
data = textscan(fid, '%s %f %f %f', 'Delimiter', ',');
fclose(fid);

sc = data{1};
t = data{2};
x = data{3};
y = data{4};

mask = strcmp(sc, TARGET_SCENARIO);
if ~any(mask)
    error('Scenario %s not found in %s', TARGET_SCENARIO, CSV_FILE);
end

t = t(mask); %#ok<NASGU>
x = x(mask);
y = y(mask);

% Load 4-way topology for context
topo = topology_4way3();

fig = figure('Name', '4-way Test Trajectory Check', ...
             'NumberTitle', 'off', ...
             'Color', 'w', ...
             'Position', [120 80 1200 520]);

tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Static overview
ax1 = nexttile;
hold(ax1, 'on');
axis(ax1, 'equal');
grid(ax1, 'on');
plot(topo.road_poly, 'FaceColor', [0.88 0.88 0.88], 'EdgeColor', [0.2 0.2 0.2]);
for k = 1:numel(topo.lane_markings)
    lm = topo.lane_markings{k};
    plot(ax1, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
end
plot(ax1, x, y, 'b-', 'LineWidth', 2.0);
plot(ax1, x(1), y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(ax1, x(end), y(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
legend(ax1, {'Road', 'Centerline', 'Test trajectory', 'Start', 'End'}, 'Location', 'best');
title(ax1, 'Static path (north then left turn)');
xlabel(ax1, 'X (m)');
ylabel(ax1, 'Y (m)');
b = topo.bounds;
xlim(ax1, b(1:2));
ylim(ax1, b(3:4));

% Animated check with car body
ax2 = nexttile;
hold(ax2, 'on');
axis(ax2, 'equal');
grid(ax2, 'on');
plot(topo.road_poly, 'FaceColor', [0.92 0.92 0.92], 'EdgeColor', [0.25 0.25 0.25]);
for k = 1:numel(topo.lane_markings)
    lm = topo.lane_markings{k};
    plot(ax2, lm(:,1), lm(:,2), '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
end
h_tail = plot(ax2, NaN, NaN, 'b-', 'LineWidth', 1.8);

car_length = 4.4;
car_width = 1.8;
car_shape_local = [
    -car_width/2, -car_length/2;
     car_width/2, -car_length/2;
     car_width/2,  car_length/2;
    -car_width/2,  car_length/2
];

h_car = patch(ax2, NaN, NaN, [0.1 0.35 0.9], 'FaceAlpha', 0.9, 'EdgeColor', [0 0 0], 'LineWidth', 1.0);
h_head = quiver(ax2, NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);

xlabel(ax2, 'X (m)');
ylabel(ax2, 'Y (m)');
xlim(ax2, b(1:2));
ylim(ax2, b(3:4));

for i = 1:numel(x)
    s = max(1, i-15);
    set(h_tail, 'XData', x(s:i), 'YData', y(s:i));

    if i < numel(x)
        dx = x(i+1) - x(i);
        dy = y(i+1) - y(i);
    else
        dx = x(i) - x(i-1);
        dy = y(i) - y(i-1);
    end

    theta = atan2(dy, dx);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    car_world = (R * car_shape_local')';
    car_world(:,1) = car_world(:,1) + x(i);
    car_world(:,2) = car_world(:,2) + y(i);

    set(h_car, 'XData', car_world(:,1), 'YData', car_world(:,2));

    dir_norm = max(hypot(dx, dy), 1e-6);
    ux = dx / dir_norm;
    uy = dy / dir_norm;
    set(h_head, 'XData', x(i), 'YData', y(i), 'UData', ux * 6, 'VData', uy * 6);

    title(ax2, sprintf('Moving car | t=%d s | x=%.1f y=%.1f', i-1, x(i), y(i)));
    drawnow;
    pause(PAUSE_SEC);
end

disp('Visualization complete. Car body should move and rotate along the route.');
