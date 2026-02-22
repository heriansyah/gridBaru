function topo = topology_t3()
%TOPOLOGY_T3 Realistic T intersection topology.

lane_width = 3.5;
lanes_per_direction = 1;
half_width = lane_width * lanes_per_direction;
road_half = half_width * 2;
arm_len = 220;

main = rect_poly_local([0, 0], arm_len * 2, road_half * 2);
stem = rect_poly_local([0, -arm_len/2], road_half * 2, arm_len);
polyT = union(main, stem);

markT = {
    [[-arm_len; arm_len], zeros(2,1)], ...
    [zeros(2,1), [-arm_len; 0]] ...
};

topo = struct();
topo.name = 'intersection_t';
topo.road_poly = polyT;
topo.lane_markings = markT;
topo.bounds = [-arm_len arm_len -arm_len arm_len * 0.7];
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
