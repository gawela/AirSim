function path = straight_segment(length, increment, starting_point, heading)

quat = rotm2quat(heading);

path = [starting_point, quat];
increment_mat = zeros(4,4);
increment_mat(1,4) = increment;
for i = 2:(length/increment)
    hom_coords_last = [heading, path(i - 1, 1:3)'; 0,0,0,1];
    new_trans = hom_coords_last(1:3,4) + hom_coords_last(1:3,1:3) * increment_mat(1:3,4);
    path = [path;
       new_trans', rotm2quat(heading)];
       
       % update heading
end
