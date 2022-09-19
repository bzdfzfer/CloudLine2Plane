clear;
clc;
close all;

basePath = '/media/bzdfzfer/Datasets/PlaneExtraction_Codes/PlaneDetectionMod2/Data/';
lidar_type = 'hdl32';
frame_id = 1;


%% load data.
ptx_file = sprintf('%s/PTX/%s/%s_%d.ptx', basePath, lidar_type, lidar_type, frame_id);
gt_geo_file = sprintf('%s/GT_GEO/%s/%s_%d.geo', basePath, lidar_type, lidar_type, frame_id);
pe_geo_file = sprintf('%s/PE_GEO/%s/%s_%d.geo', basePath, lidar_type, lidar_type, frame_id);

point_cloud = load(ptx_file);
[gt_plane_params, gt_planes_ptIdxs]=loadGeo(gt_geo_file);
[pe_plane_params, pe_planes_ptIdxs] = loadGeo(pe_geo_file);
% [rmses, rmse_avg] = calcPlaneRMSE(point_cloud, plane_params, planes_ptIdxs);
% 
% rmse_avg 

%% find correspondences.
correspondences = [];
for i=1:size(pe_plane_params,1)
    gt_plane_idx = findCorrespondences(gt_plane_params, gt_planes_ptIdxs, ...
        pe_plane_params(i,:), pe_planes_ptIdxs{i});
    if size(gt_plane_idx,1)>0
        correspondences = [correspondences; [i, gt_plane_idx]];
    end
end

%% visualize planes.
figure(1);
hold on;
for i=1:size(correspondences, 1)
    plotPlane(pe_plane_params(correspondences(i,1),:), [0,0,4]);
    plotPlane(gt_plane_params(correspondences(i,2),:), [0,0,0]);
    pe_center = pe_plane_params(correspondences(i,1),4:6) + [0,0,4];
    gt_center = gt_plane_params(correspondences(i,2),4:6) + [0,0,0];
    line_pts = [pe_center; gt_center];
    line(line_pts(:,1), line_pts(:,2), line_pts(:,3), 'color', 'k');
end
axis equal;
grid on;

% 

