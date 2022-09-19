function dists = getPointsPlaneDists(plane_points, plane_params)
% input: Kx3 matrix. 
% output: Kx1 vector.
centroid = plane_params(1:3);
normal = plane_params(4:6);

neg_pt_dot_norm = -dot(centroid, normal);

normal = normal';
dists = plane_points*normal + neg_pt_dot_norm;


end