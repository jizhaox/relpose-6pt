function [R_extrinsic_para2, T_extrinsic_para2] = generic_interface_pc(R_extrinsic_para, T_extrinsic_para, match_type)
%% unified interface

[match_info, n_cam] = match_type_gcam_pc(match_type); 

R_extrinsic_para2 = zeros(3,3,12);
T_extrinsic_para2 = zeros(3,12);
for ii = 1:6
    idx1 = match_info{ii}.idx1;
    idx2 = match_info{ii}.idx2;
    R_extrinsic_para2(:,:,ii*2-1) = R_extrinsic_para(:,:,idx1);
    R_extrinsic_para2(:,:,ii*2) = R_extrinsic_para(:,:,idx2);
    T_extrinsic_para2(:,ii*2-1) = T_extrinsic_para(:,idx1);
    T_extrinsic_para2(:,ii*2) = T_extrinsic_para(:,idx2);
end
