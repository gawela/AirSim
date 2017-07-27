function path = curve_segment(starting_point, heading, increment_angle, increment_dist, total_angle)

quat = rotm2quat(heading);

path = [starting_point, quat];
increment_mat = zeros(4,4);
increment_mat(1:3,1:3) = [cos(pi * increment_angle/180), -sin(pi * increment_angle/180), 0;
                 sin(pi * increment_angle/180), cos(pi * increment_angle/180), 0;
                 0,0,1];
increment_mat(1,4) = increment_dist;

for i = 2:(total_angle/norm(increment_angle) + 1)
    heading = heading * increment_mat(1:3,1:3);
    hom_coords_last = [heading, path(i - 1, 1:3)'; 0,0,0,1];
    new_trans = hom_coords_last(1:3,4) + hom_coords_last(1:3,1:3) * increment_mat(1:3,4);
    path = [path;
       new_trans', rotm2quat(heading)];
end