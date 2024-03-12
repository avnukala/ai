function vel = swarm(rad_rep, rad_ori, rad_att, x, N, dxi)
% FILE: swarm.m implements a Boids-like behavior
%
% DESCRIPTION:
% Boids-like repulsion-orientation-attraction behavior based loosely on the 
% behavior described by Couzin et al. in the Collective Memory paper. 
%
% INPUTS:
% rad_rep - radius of repulsion
% rad_ori - radius of orientation
% rad_att - radius of attraction
% x - matrix containing the pose of all the robots; x(1, ii) is the
% position of robot ii along the horizontal axis; x(2, ii) is the position
% of robot ii along the vertical axis; x(3, ii) is the heading of robot ii
% in radians. Easier alternative to dealing with radians is to use
% dxi(:,ii) instead, which is the heading or velocity of robot ii, as a
% vector
% blind_neighbors - matrix tracking the robots in a robot's blind spot
% blind_neighbors not used in the Assignment 
% neighbors - NxN matrix; entry (ii, jj) is 1 if agents ii and jj are
% neighbors; otherwise, entry is 0
% neighbors not used in the Assignment
% N - the number of robots in the swarm
% dxi - the current velocity of the robots (2 x N vector); dxi(1, ii) is
% robot ii's velocity component along the horizontal axis, while dxi(2, ii) 
% is robot ii's velocity component along the vertical axis 
%
% OUTPUTS:
% vel - the resulting velocity of the robots (2 x N vector)
%
% TODO:
% Return the velocity (i.e., heading) that emerges from implementing 
% repulsion, orientation, and attracton interaction rules

%% Authors: Safwan Alam, Musad Haque - 2018
%%%%%%%%%%%%%

% dist(ii, jj) is the distance between robots ii and jj
dist = distances_from_others(x, N); 

% Random jitter movement <-- REMOVE
% dxi = -1 + 2*rand(2, 10); %<-- REMOVE!!!

repulsion_direction = zeros(2, N);
orientation_direction = zeros(2, N);
attraction_direction = zeros(2, N);

for ii = 1:N
    for jj = 1:N
        if ii == jj
            continue
        end 
        distance = dist(ii, jj);
        if distance < rad_rep
            r_ij = x(1:2, jj) - x(1:2, ii);
            repulsion_direction(:, ii) = repulsion_direction(:, ii) - r_ij / norm(r_ij);
        elseif norm(dxi(:, jj)) ~= 0 && rad_rep <= distance && distance < rad_ori
            d_j = dxi(:, jj) / norm(dxi(:, jj));
            orientation_direction(:, ii) = orientation_direction(:, ii) + d_j;
        elseif rad_ori <= distance && distance < rad_att
            r_ij = x(1:2, jj) - x(1:2, ii);
            attraction_direction(:, ii) = attraction_direction(:, ii) + r_ij / norm(r_ij);
        end
    end
end

% Accumulate/aggregate the resulting headings in some fashion, depending on
% how you implement the three behaviors above.

for i = 1:N
    prev_dxi = dxi(:, i);
    repulsion_state = ~all(repulsion_direction(:, i) == 0);
    orientation_state = ~all(orientation_direction(:, i) == 0);
    attraction_state = ~all(attraction_direction(:, i) == 0);

    if repulsion_state
        dxi(:, i) = repulsion_direction(:, i);
    elseif orientation_state && attraction_state
        dxi(:, i) = (orientation_direction(:, i) + attraction_direction(:, i)) / 2;
    elseif orientation_state
        dxi(:, i) = orientation_direction(:, i);
    elseif attraction_state
        dxi(:, i) = attraction_direction(:, i);
    else 
        dxi(:, i) = prev_dxi;
    end
end

% Return the resulting velocity
vel = dxi;

end


