% Corners of the map. Fly a rectangle for now.

starting_point = [-126, -144, 2];
checkpoint_1 = [130, -144, 2];
checkpoint_2 = [130, 112, 2];
checkpoint_3 = [-126, 112, 2];
end_point = [-126, -144, 2];

waypoints = [starting_point; checkpoint_1; checkpoint_2; checkpoint_3; end_point];

orientation_start = [1, 0, 0, 0];
orientation_1 = [1, 0, 0, 0];
orientation_2 = [1, 0, 0, 0];
orientation_3 = [1, 0, 0, 0];
orientation_end = [1, 0, 0, 0];

current_waypoint = waypoints(1,:);
incremental_waypoints = [];
for i = 2:size(waypoints,1)
    % Calculate heading (north, east, south, west)
    delta_wp = waypoints(i,1:2) - current_waypoint(1:2);
    % Count how many 20 cm steps (hence the factor 5).
    num_steps = (norm(delta_wp) - 1.15) * 5;
    
    axis = 0;
    switch_case = 0;
    if delta_wp(1) > 0
        rotm = [1,0,0;0,1,0;0,0,1];
        quat = rotm2quat(rotm);
        axis = 1;
        counter_axis = 2;
        switch_case = 1;
        % positive x
    elseif delta_wp(1) < 0
        rotm = [-1,0,0;0,-1,0;0,0,1];
        quat = rotm2quat(rotm);
        axis = 1;
        counter_axis = 2;
        switch_case = 2;
        % negative x
    elseif delta_wp(2) > 0
        rotm = [0,-1,0;1,0,0;0,0,1];
        quat = rotm2quat(rotm);
        axis = 2;
        counter_axis = 1;
        switch_case = 3;
        % positive y
    elseif delta_wp(2) < 0
        rotm = [0,1,0;-1,0,0;0,0,1];
        quat = rotm2quat(rotm);
        axis = 2;
        counter_axis = 1;
        switch_case = 4;
        % negative y
    else
        % same point.
        break
    end
    
    % Write the 20 cm steps in wp list.
    if switch_case == 1 || switch_case == 3
        stretch = current_waypoint(axis):0.2:(waypoints(i,axis) - 1.2);
    elseif switch_case == 2 || switch_case == 4
        stretch = current_waypoint(axis):0.2:(waypoints(i,axis) + 1.2);
    else 
        break;
    end

    new_wps = zeros(length(stretch), 7);
    new_wps(:,axis) = stretch';
    new_wps(:,counter_axis) = repmat(current_waypoint(counter_axis), length(stretch),1);
    new_wps(:,3) = repmat(current_waypoint(3), length(stretch),1);
    new_wps(:,4) = repmat(quat(1), length(stretch),1);
    new_wps(:,5) = repmat(quat(2), length(stretch),1);
    new_wps(:,6) = repmat(quat(3), length(stretch),1);
    new_wps(:,7) = repmat(quat(4), length(stretch),1);
    
    % Add curve segments to wp list in 10° steps if more wps to go.
    % Perturb rotation matrix to get nextz orientation.
    increment_right = [cos(pi * 10/180), -sin(pi * 10/180), 0;
                 sin(pi * 10/180), cos(pi * 10/180), 0;
                 0,0,1];
    increment_left = [cos(pi * -10/180), -sin(pi * -10/180), 0;
                 sin(pi * -10/180), cos(pi * -10/180), 0;
                 0,0,1];
    if i < size(waypoints,1)
        % Curve left or right?
        edge_delta = waypoints(i+1,:) - waypoints(i,:);
        if edge_delta(1) > 0 && edge_delta(2) > 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        elseif edge_delta(1) > 0 && edge_delta(2) < 0
            
        end
        
        
        for j = 1:9
            rotm = rotm * increment;
            quat = rotm2quat(rotm);
            if switch_case == 1
                loc = 1
            end
        end
    end

    
    incremental_waypoints = [incremental_waypoints; new_wps];
    % Update current wapoint for next itteration
    current_waypoint = waypoints(i,:);
    if switch_case == 1 || switch_case == 3
        current_waypoint(axis) = current_waypoint(axis) + 1.2;
    elseif switch_case == 2 || switch_case == 4
        current_waypoint(axis) = current_waypoint(axis) - 1.2;
    else 
        break;
    end
    
end


% Interpolate path between corners in 10 cm steps.
path = [];
stretch = 0:0.1:256;
path = [path; [stretch', repmat(0, length(stretch), 1), repmat(0, length(stretch), 1), repmat(1, length(stretch), 1), repmat(0, length(stretch), 1), repmat(0, length(stretch), 1), repmat(0, length(stretch), 1)]];


% GPSS: 47.6415, lon=-122.14, alt=122.146


csvwrite('C:/Users/gawela/Documents/neighbourhood/waypoints_neighbourhood_2.csv', incremental_waypoints);