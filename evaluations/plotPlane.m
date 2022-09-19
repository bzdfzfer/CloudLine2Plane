function plotPlane(plane_param, offset)

color = plane_param(1:3);
centroid = plane_param(4:6);
basisU = plane_param(10:12);
basisV = plane_param(13:15);

rect_pts = [centroid - basisU - basisV;
            centroid - basisU + basisV;            
            centroid + basisU + basisV;
            centroid + basisU - basisV];

rect_pts = rect_pts + repmat(offset, 4, 1);
fill3(rect_pts(:,1), rect_pts(:,2), rect_pts(:,3), color);
% surf(rect_pts(:,1), rect_pts(:,2), rect_pts(:,3), color);