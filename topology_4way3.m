function topo = topology_4way3()
%TOPOLOGY_4WAY3 Realistic 4-way intersection topology.

lane_width = 3.5;
lanes_per_direction = 1;
half_width = lane_width * lanes_per_direction;
road_half = half_width * 2;
arm_len = 220;

vert = rect_poly_local([0, 0], road_half * 2, arm_len * 2);
horz = rect_poly_local([0, 0], arm_len * 2, road_half * 2);
poly4 = union(vert, horz);

mark4 = {
    [zeros(2,1), [-arm_len; arm_len]], ...
    [[-arm_len; arm_len], zeros(2,1)] ...
};

topo = struct();
topo.name = 'intersection_4way';
topo.road_poly = poly4;
topo.lane_markings = mark4;
topo.bounds = [-arm_len arm_len -arm_len arm_len];
topo.meta = struct('lane_width', lane_width, ...
                   'lanes_per_direction', lanes_per_direction);
end

function p = rect_poly_local(center, width, height)
    cx = center(1);
    cy = center(2);
    x = [cx - width/2, cx + width/2, cx + width/2, cx - width/2];
    y = [cy - height/2, cy - height/2, cy + height/2, cy + height/2];
    p = polyshape(x, y);
end
