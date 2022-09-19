function normal = getPlaneNormal(plane_pts, centroid)

numPoints = size(plane_pts, 1);
centers = repmat(centroid, numPoints, 1);
pts_centered = plane_pts - centers;

cov_mat = pts_centered'*pts_centered/numPoints;

xx = cov_mat(1,1);
yy = cov_mat(2,2);
zz = cov_mat(3,3);
xy = cov_mat(1,2);
xz = cov_mat(1,3);
yz = cov_mat(2,3);

weighted_dir = zeros(1,3);

det_x = yy*zz - yz*yz;
axis_dir1 = [det_x, xz*yz-xy*zz, xy*yz-xz*yy];
weight1 = det_x*det_x;
weighted_dir = weighted_dir + axis_dir1*weight1;

det_y = xx*zz - xz*xz;
axis_dir2 = [xz*yz - xy*zz, det_y, xy*xz - yz*xx];
weight2 = det_y * det_y;
if dot(weighted_dir,axis_dir2)< 0.0 
    weight2 = -weight2;
end
weighted_dir = weighted_dir + axis_dir2*weight2;

det_z = xx*yy - xy*xy;
axis_dir3 = [xy*yz - xz*yy, xy*xz - yz*xx, det_z];
weight3 = det_z * det_z;
if dot(weighted_dir, axis_dir3)<0.0
    weight3 = -weight3;
end
weighted_dir =weighted_dir + axis_dir3*weight3;

weighted_dir = weighted_dir/norm(weighted_dir);

plane_d = -dot(weighted_dir, centroid);

if plane_d < 0
    weighted_dir = - weighted_dir;
end

normal = weighted_dir;
