function topo = topology_y3()
%TOPOLOGY_Y3 Realistic Y intersection topology (true Y shape).

lane_width = 3.5;
lanes_per_direction = 1;
half_width = lane_width * lanes_per_direction;
road_half = half_width * 2;

split_pt = [0, 0];
stem_bottom = [0, -220];
left_tip = [-170, 170];
right_tip = [170, 170];

stemY = corridor_from_segment_local(stem_bottom, split_pt, road_half * 2);
leftArm = corridor_from_segment_local(split_pt, left_tip, road_half * 2);
rightArm = corridor_from_segment_local(split_pt, right_tip, road_half * 2);
polyY = union(union(stemY, leftArm), rightArm);

markY = {
    [stem_bottom; split_pt], ...
    [split_pt; left_tip], ...
    [split_pt; right_tip] ...
};

topo = struct();
topo.name = 'intersection_y';
topo.road_poly = polyY;
topo.lane_markings = markY;
topo.bounds = [-220 220 -220 220];
topo.meta = struct('lane_width', lane_width, ...
                   'lanes_per_direction', lanes_per_direction);
end

function p = corridor_from_segment_local(p1, p2, width)
    v = p2 - p1;
    n = norm(v);
    if n < 1e-9
        p = polyshape();
        return;
    end

    t = v / n;
    perp = [-t(2), t(1)];
    hw = width / 2;

    a = p1 + perp * hw;
    b = p2 + perp * hw;
    c = p2 - perp * hw;
    d = p1 - perp * hw;

    p = polyshape([a(1) b(1) c(1) d(1)], [a(2) b(2) c(2) d(2)]);
end
