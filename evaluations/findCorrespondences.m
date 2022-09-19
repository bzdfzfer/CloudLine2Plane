function [best_plane_idx] = findCorrespondences(gt_plane_params,...
                                                gt_plane_ptsIdxs, ...
                                                est_plane_param,...
                                                est_plane_inliers)

                                            
gt_planes_num = size(gt_plane_params,1);
numEstPlanePoints = length(est_plane_inliers);

countInliers = zeros(gt_planes_num+1, 1);
for i=1:numEstPlanePoints
    inlier_idx = est_plane_inliers(i);
    found = false;
    for j=1:size(gt_plane_ptsIdxs,1)
        gt_plane_inliers = gt_plane_ptsIdxs{j};
        found_idx = find(gt_plane_inliers==inlier_idx);
        if size(found_idx,1)>0
            countInliers(j) = countInliers(j) + 1;
            found = true;
            break;
        end
    end
    
    if found==false
        countInliers(gt_planes_num+1) = countInliers(gt_planes_num+1)+1;
    end
end

[max_val, max_idx] = max(countInliers);

if max_idx == gt_planes_num+1 || max_val < numEstPlanePoints/2 || ...
        isPlaneNotEquivalent(gt_plane_params(max_idx,:), est_plane_param)
    best_plane_idx = [];
else
    best_plane_idx = max_idx;    
end

end

function sim_flag = isPlaneNotEquivalent(plane1_param, plane2_param)
    cos_val= abs(dot(plane1_param(7:9), plane2_param(7:9)));
    if cos_val<0.86
        sim_flag = true;
    else 
        sim_flag = false;
    end
end