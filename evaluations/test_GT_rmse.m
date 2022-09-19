clear;
clc;
close all;

% basePath = '/media/bzdfzfer/Datasets/PlaneExtraction_Codes/PlaneDetectionMod/Data/';
% lidar_type = 'vlp16';
% frame_id = 869;

% basePath = '/media/bzdfzfer/Datasets/PlaneExtraction_Codes/PlaneDetectionMod2/Data/';
% lidar_type = 'vlp16/vlp16';
% frame_id = 1;

basePath = '/media/bzdfzfer/Datasets/PlaneExtraction_Codes/PlaneDetectionMod2/Data/';
lidar_type = 'hdl32';
frame_id = 1;

ptx_file = sprintf('%s/PTX/%s/%s_%d.ptx', basePath, lidar_type, lidar_type, frame_id);
geo_file = sprintf('%s/GT_GEO/%s/%s_%d.geo', basePath, lidar_type, lidar_type, frame_id);


point_cloud = load(ptx_file);
[plane_params, planes_ptIdxs]=loadGeo(geo_file);

[rmses, rmse_avg] = calcPlaneRMSE(point_cloud, plane_params, planes_ptIdxs);

rmse_avg 

% visualize plane cloud.
figure(1);
hold on;
for i=1:size(planes_ptIdxs,1)
    
%     if rmses(i) < 0.5
%         continue;
%     end
    
    col = randn(1,3);
    col = abs(col)/norm(col);
    
    plane_points = point_cloud(planes_ptIdxs{i},:);
    plot3(plane_points(:,1), plane_points(:,2), plane_points(:,3),'.', 'color', col);

    plot3(plane_params(i,4), plane_params(i,5), plane_params(i,6), 'g*', 'markersize', 5);
    
    
%     if rmses(i) > 0.01
%         m_x = mean(plane_points(:,1));
%         m_y = mean(plane_points(:,2));
%         m_z = mean(plane_points(:,3));
%         new_centroid = [m_x, m_y, m_z];
%         plane_params(i,4:6) = new_centroid;
%         new_normal = getPlaneNormal(plane_points, new_centroid);
%         plane_params(i,7:9) = new_normal;
%         pt_pl_dists = getPointsPlaneDists(plane_points, plane_params(i,4:9));
%         
%         fprintf('before optimization %f \n', rmses(i));
%         % root mean square error.
%         rmses(i) = sqrt(mean(pt_pl_dists.^2));
%         fprintf('after optimization %f \n', rmses(i));
% 
%         fprintf('center: (%f, %f, %f) \n', new_centroid(1), new_centroid(2), new_centroid(3));
%         fprintf('normal: (%f, %f, %f) \n', new_normal(1), new_normal(2), new_normal(3));        
%         fprintf('plane d: %f\n', dot(new_centroid, new_normal));
%         
%         plot3(plane_points(:,1), plane_points(:,2), plane_points(:,3),'o', 'color', [0,0,0]);
%     end
    
    % plot center points.
%     plot3(plane_params(i,4), -plane_params(i,6), plane_params(i,5), 'r*', 'markersize', 5);
    plot3(plane_params(i,4), plane_params(i,5), plane_params(i,6), 'r*', 'markersize', 5);
    pause(0.001);
end

rmse_avg = mean(rmses)

axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
% view(45,45);