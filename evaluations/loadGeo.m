function [plane_params, plane_pts_indices]=loadGeo(filename)

%% Open file.
% Try to open the file.
fid = fopen(filename, 'rb');
if fid == -1
    error(['Cannot read ', filename, '.'])
end

%% Read plane data.
numCircles = fread(fid, 1,  'uint64');
% fprintf('circles num: %d\n', numCircles);

numPlanes = fread(fid, 1,  'uint64');
fprintf('planes num: %d\n', numPlanes);

% plane param: color, center, normal, basisU, basisV, 
plane_params = zeros(numPlanes, 5*3);

% plane pts indices.
plane_pts_indices = cell(numPlanes, 1);

for i=1:numPlanes
    color = fread(fid, 3, 'float');
    center = fread(fid, 3, 'float');
    normal = fread(fid, 3, 'float');
    basisU = fread(fid, 3, 'float');
    basisV = fread(fid, 3, 'float');
    numInliers = fread(fid, 1, 'uint64');
    
%     fprintf( 'center: (%f, %f, %f), normal: (%f, %f, %f)\n', ...
%         center(1), center(2), center(3), normal(1), normal(2), normal(3));
%     fprintf( 'basisU: (%f, %f, %f), basisV: (%f, %f, %f)\n', ...
%         basisU(1), basisU(2), basisU(3), basisV(1), basisV(2), basisV(3));
%     fprintf( 'Inliers num: %d \n', numInliers);
    
    InliersIndices = fread(fid, numInliers, 'uint64');
    
%     fprintf( 'InliersIndices size: %d \n', size(InliersIndices,1));
    
%     tx = center(1);
%     ty = -center(3);
%     tz = center(2);
%     tx = center(1);
%     ty = center(2);
%     tz = center(3);
%     nx = normal(1);
%     ny = -normal(3);
%     nz = normal(2);
% 
%     center = [tx; ty; tz];
%     normal = [nx; ny; nz];

    center = coord3DConvert(center);
    normal = coord3DConvert(normal);
    basisU = coord3DConvert(basisU);
    basisV = coord3DConvert(basisV);
    plane_params(i,:) = [color; center; normal; basisU; basisV]';
    plane_pts_indices{i} = InliersIndices+1;
end


%% Close file.
fclose(fid);
end

function vec3_data_out = coord3DConvert(vec3_data)
    vec3_data_out = zeros(size(vec3_data));
    tmpx = vec3_data(1);
    tmpy = -vec3_data(3);
    tmpz = vec3_data(2);
    vec3_data_out(1) = tmpx;
    vec3_data_out(2) = tmpy;
    vec3_data_out(3) = tmpz;
end
