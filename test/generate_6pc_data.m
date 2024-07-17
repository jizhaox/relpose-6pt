function [Image1, Image2, R_extrinsic_para, T_extrinsic_para, R_gt, cay_gt, t_gt, match_info] = generate_6pc_data(match_type)

%% Three types of point correspondence
%% generic-cam correspondences contain
% (1) 1 correspondence: (cam1 at time i)  <--> (cam2 at time j)
% (2) 1 correspondence: (cam3 at time i)  <--> (cam4 at time j) 
% (3) 1 correspondence: (cam5 at time i)  <--> (cam6 at time j)
% (4) 1 correspondence: (cam7 at time i)  <--> (cam8 at time j)
% (5) 1 correspondence: (cam9 at time i)  <--> (cam10 at time j)
% (6) 1 correspondence: (cam11 at time i) <--> (cam12 at time j)
%% inter-cam correspondences contain
% (1) 3 correspondences: (cam1 at time i) <--> (cam2 at time j)
% (2) 3 correspondences: (cam2 at time i) <--> (cam1 at time j)
%% intra-cam correspondences contain
% (1) 3 correspondences: (cam1 at time i) <--> (cam1 at time j)
% (2) 3 correspondences: (cam2 at time i) <--> (cam2 at time j)

[match_info, n_cam] = match_type_gcam_pc(match_type);
n_point = 6;

%% define random extrinsic parameters
cam_body_rotation = cell(n_cam, 1);
cam_body_offset = cell(n_cam, 1);
R_cam = cell(n_cam, 1);
t_cam = cell(n_cam, 1);
T_body_cam = cell(n_cam, 1);
for ii = 1:n_cam
    cay = rand(3, 1);
    cam_body_rotation{ii} = cayley_rotation(cay);
    cam_body_offset{ii} = rand(3, 1);
    
    R_cam{ii} = cam_body_rotation{ii}';
    t_cam{ii} = -R_cam{ii}*cam_body_offset{ii};
    % transformation from body reference to perspective camera references
    T_body_cam{ii} = [R_cam{ii} t_cam{ii}; 0 0 0 1];
end

%% define relative pose
cay = rand(3, 1);
R_gt = cayley_rotation(cay);
q = rotm2quat(R_gt);
cay_gt = q(2:4)/q(1);
cay_gt = cay_gt(:);

t_gt = rand(3, 1);

% transformation from body reference at time i to time j
T_gt = [R_gt t_gt; 0 0 0 1];

%% generating random scene points
points_all = cell(n_point, 1);
for ii = 1:n_point
    PT = rand(3, 1);
    points_all{ii} = struct('point', PT);
end

%% extract point observations
% images at time i
x_i = cell(n_point, n_cam);
for ii = 1:n_point
    PT = points_all{ii}.point;
    for jj = 1:n_cam
        tmp = R_cam{jj}*PT+t_cam{jj};
        x_i{ii,jj} = tmp/norm(tmp);
    end
end
% images at time j
Rc_j = cell(n_cam, 1);
tc_j = cell(n_cam, 1);
for ii = 1:n_cam
    tmp = T_body_cam{ii}*T_gt;
    Rc_j{ii} = tmp(1:3,1:3);
    tc_j{ii} = tmp(1:3,4);
end
x_j = cell(n_point, n_cam);
for ii = 1:n_point
    PT = points_all{ii}.point;
    for jj = 1:n_cam
        tmp = Rc_j{jj}*PT+tc_j{jj};
        x_j{ii,jj} = tmp/norm(tmp);
    end
end

%% construct observations
R_extrinsic_para = zeros(3,3,n_cam);
T_extrinsic_para = zeros(3, n_cam);
for ii = 1:n_cam
    R_extrinsic_para(:,:,ii) = cam_body_rotation{ii};
    T_extrinsic_para(:,ii) = cam_body_offset{ii};
end
Image1 = zeros(3, n_point);
Image2 = zeros(3, n_point);

for ii = 1:n_point
    idx1 = match_info{ii}.idx1;
    idx2 = match_info{ii}.idx2;
    
    Image1(:,ii) = x_i{ii,idx1};
    Image2(:,ii) = x_j{ii,idx2};
end

