clear;
clc;
close all;

% Choose: 'all', '4way', 't', 'y'
TOPOLOGY_VIEW = 'all';

switch lower(TOPOLOGY_VIEW)
    case '4way'
        topologies = topology_4way3();
    case 't'
        topologies = topology_t3();
    case 'y'
        topologies = topology_y3();
    otherwise
        topologies = topology_library3();
end

if numel(topologies) == 1
    fig_w = 520;
else
    fig_w = 1400;
end

fig = figure('Name', 'SmartGrid3 Topologies', ...
             'NumberTitle', 'off', ...
             'Color', 'w', ...
             'Position', [120 80 fig_w 480]);

tiledlayout(fig, 1, numel(topologies), 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:numel(topologies)
    ax = nexttile;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');

    pg = topologies(i).road_poly;
    plot(pg, 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 1.2);

    for k = 1:numel(topologies(i).lane_markings)
        m = topologies(i).lane_markings{k};
        plot(ax, m(:,1), m(:,2), '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.0);
    end

    b = topologies(i).bounds;
    xlim(ax, b(1:2));
    ylim(ax, b(3:4));
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    title(ax, strrep(topologies(i).name, '_', ' '), 'Interpreter', 'none');
end

disp('Topology visualization complete.');
