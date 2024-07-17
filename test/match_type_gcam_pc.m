function [match_info, n_cam] = match_type_gcam_pc(match_type)

if nargin<1
    match_type = 'generic';
end

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

if strcmp(match_type, 'generic')
    % PCs of 12 cameras
    n_cam = 12;
    match_info{1} = struct('idx1', 1, 'idx2', 2);
    match_info{2} = struct('idx1', 3, 'idx2', 4);
    match_info{3} = struct('idx1', 5, 'idx2', 6);
    match_info{4} = struct('idx1', 7, 'idx2', 8);
    match_info{5} = struct('idx1', 9, 'idx2', 10);
    match_info{6} = struct('idx1', 11, 'idx2', 12);
elseif strcmp(match_type, 'inter')
    % PCs of 2 cameras
    n_cam = 2;
    match_info{1} = struct('idx1', 1, 'idx2', 2);
    match_info{2} = struct('idx1', 1, 'idx2', 2);
    match_info{3} = struct('idx1', 1, 'idx2', 2);
    match_info{4} = struct('idx1', 2, 'idx2', 1);
    match_info{5} = struct('idx1', 2, 'idx2', 1);
    match_info{6} = struct('idx1', 2, 'idx2', 1);
elseif strcmp(match_type, 'intra')
    % PCs of 2 cameras
    n_cam = 2;
    match_info{1} = struct('idx1', 1, 'idx2', 1);
    match_info{2} = struct('idx1', 1, 'idx2', 1);
    match_info{3} = struct('idx1', 1, 'idx2', 1);
    match_info{4} = struct('idx1', 2, 'idx2', 2);
    match_info{5} = struct('idx1', 2, 'idx2', 2);
    match_info{6} = struct('idx1', 2, 'idx2', 2);
else
    error('match type error!')
end

