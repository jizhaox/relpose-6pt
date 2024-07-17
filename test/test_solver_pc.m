clear;
addpath('../solver')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--------------6-point method & generic case--------------') 
%% generate synthetic data
match_type = 'generic';
[Image1, Image2, R_cam, t_cam, R_gt, cay_gt, t_gt, match_info] = generate_6pc_data(match_type);

%% check data
% For noise-free data, the residual of epipolar constraint should be small (Eq.(1) in the paper)
err_epipolar = zeros(6, 1);
Tf_gt = [R_gt t_gt; 0 0 0 1];
n_cam = size(R_cam,3);
Tf_cam = cell(n_cam, 1);
for ii = 1:n_cam
    Tf_cam{ii} = [R_cam(:,:,ii), t_cam(:,ii); 0, 0, 0, 1];
end
for ii = 1:6
    idx1 = match_info{ii}.idx1;
    idx2 = match_info{ii}.idx2;
    x1 = Image1(:, ii);
    x2 = Image2(:, ii);
    % relative pose between time i and time j
    Hij = Tf_cam{idx2}\Tf_gt*(Tf_cam{idx1});
    Rij = Hij(1:3,1:3);
    tij = Hij(1:3,4);
    % epipolar geometry of PC
    x = tij;
    skew_tij=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
    E =skew_tij*Rij;
    err_epipolar(ii) = x2'*E*x1;
end
disp(['The maximum residual of epipolar geometry: ' num2str(max(abs(err_epipolar(:))))]);

%% run solver
% 64 solutions
[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_generic_6pc(Image1, Image2, R_cam, t_cam);
cay_sol = find_solution(cay_sols, cay_gt);
t_sol = find_solution(t_sols, t_gt);
cay_sol, t_sol, cay_gt, t_gt


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--------------6-point method & inter-camera case--------------')
%% generate synthetic data
match_type = 'inter';
[Image1, Image2, R_cam, t_cam, R_gt, cay_gt, t_gt, match_info] = generate_6pc_data(match_type);

%% check data
% For noise-free data, the residual of epipolar constraint should be small (Eq.(1) in the paper)
err_epipolar = zeros(6, 1);
Tf_gt = [R_gt t_gt; 0 0 0 1];
n_cam = size(R_cam,3);
Tf_cam = cell(n_cam, 1);
for ii = 1:n_cam
    Tf_cam{ii} = [R_cam(:,:,ii), t_cam(:,ii); 0, 0, 0, 1];
end
for ii = 1:6
    idx1 = match_info{ii}.idx1;
    idx2 = match_info{ii}.idx2;
    x1 = Image1(:, ii);
    x2 = Image2(:, ii);
    % relative pose between time i and time j
    Hij = Tf_cam{idx2}\Tf_gt*(Tf_cam{idx1});
    Rij = Hij(1:3,1:3);
    tij = Hij(1:3,4);
    % epipolar geometry of PC
    x = tij;
    skew_tij=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
    E =skew_tij*Rij;
    err_epipolar(ii) = x2'*E*x1;
end
disp(['The maximum residual of epipolar geometry: ' num2str(max(abs(err_epipolar(:))))]);

%% run solver
% set the 5th parameter as 0 (default), 48 solutions
[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_inter_6pc(Image1, Image2, R_cam, t_cam, 0);
cay_sol = find_solution(cay_sols, cay_gt);
t_sol = find_solution(t_sols, t_gt);
cay_sol, t_sol, cay_gt, t_gt

% set the 5th parameter as 1, 56 solutions.
% according to our evaluation, 56-solution configuration has better numerical stability
[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_inter_6pc(Image1, Image2, R_cam, t_cam, 1);
cay_sol = find_solution(cay_sols, cay_gt);
t_sol = find_solution(t_sols, t_gt);
cay_sol, t_sol, cay_gt, t_gt


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--------------6-point method & intra-camera case--------------');
match_type = 'intra';
[Image1, Image2, R_cam, t_cam, R_gt, cay_gt, t_gt, match_info] = generate_6pc_data(match_type);

%% check data
% For noise-free data, the residual of epipolar constraint should be small (Eq.(1) in the paper)
err_epipolar = zeros(6, 1);
Tf_gt = [R_gt t_gt; 0 0 0 1];
n_cam = size(R_cam,3);
Tf_cam = cell(n_cam, 1);
for ii = 1:n_cam
    Tf_cam{ii} = [R_cam(:,:,ii), t_cam(:,ii); 0, 0, 0, 1];
end
for ii = 1:6
    idx1 = match_info{ii}.idx1;
    idx2 = match_info{ii}.idx2;
    x1 = Image1(:, ii);
    x2 = Image2(:, ii);
    % relative pose between time i and time j
    Hij = Tf_cam{idx2}\Tf_gt*(Tf_cam{idx1});
    Rij = Hij(1:3,1:3);
    tij = Hij(1:3,4);
    % epipolar geometry of PC
    x = tij;
    skew_tij=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
    E =skew_tij*Rij;
    err_epipolar(ii) = x2'*E*x1;
end
disp(['The maximum residual of epipolar geometry: ' num2str(max(abs(err_epipolar(:))))]);

%% run solver
% 48 solutions
[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_intra_6pc(Image1, Image2, R_cam, t_cam);
cay_sol = find_solution(cay_sols, cay_gt);
t_sol = find_solution(t_sols, t_gt);
cay_sol, t_sol, cay_gt, t_gt
