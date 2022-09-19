function [rmses, rmse_avg]=calcPlaneRMSE(pointcloud, plane_params, plane_indices)
numPlanes = size(plane_indices,1);
rmses = zeros(numPlanes, 1);


for i=1:numPlanes

    plane_center = plane_params(i,4:6);
    plane_norm = plane_params(i,7:9);
    fprintf('center: (%f, %f, %f) \n', plane_center(1), plane_center(2), plane_center(3));
    fprintf('normal: (%f, %f, %f) \n', plane_norm(1), plane_norm(2), plane_norm(3));
    fprintf('plane d: %f\n', dot(plane_norm, plane_center));
    % Kx3 
    plane_points = getPlanePoints(pointcloud, plane_indices{i});
    % Kx1
    pt_pl_dists = getPointsPlaneDists(plane_points, plane_params(i,4:9));
    % root mean square error.
    rmses(i) = sqrt(mean(pt_pl_dists.^2));
    
    if rmses(i) > 0.5
        fprintf('rmse %f \n', rmses(i));
    end
end

rmse_avg = mean(rmses);

end

function pl_points = getPlanePoints(point_cloud, pts_indices)
%     numPlanePoints = size(pts_indices,1);
%     pl_points = zeros(numPlanePoints, 3);
    numPoints = size(point_cloud, 1);
    
%     out_of_range_u = find(pts_indices>numPoints);
%     out_of_range_d = find(pts_indices<1);
%     
%     if length(out_of_range_u)>0
%         fprintf('out of range(up)\n');
%     end
%     
%     if length(out_of_range_d)>0
%         fprintf('out of range (down)\n');
%     end
    
%     pl_points = point_cloud(pts_indices, :);
    pl_points = [];
    for i=1:size(pts_indices,1)
        pt = point_cloud(pts_indices(i), :);
        
        pt_dist = pt(1)*pt(1) + pt(2)*pt(2) + pt(3)*pt(3);
        
        pt_dist = sqrt(pt_dist);
        
        if pt_dist<0.001
            fprintf('% idx, point: (%f, %f, %f)\n', ...
                pts_indices(i), pt(1), pt(2), pt(3));
            continue;
        end
        
        pl_points = [pl_points; pt];   
    end
end



