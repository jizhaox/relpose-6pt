
eigen_dir = '/usr/include/eigen3';

tic; mex(['-I"' eigen_dir '"'],'-O','solver_trans_generic_6pc.cpp'); toc
tic; mex(['-I"' eigen_dir '"'],'-O','solver_trans_inter_6pc.cpp'); toc
tic; mex(['-I"' eigen_dir '"'],'-O','solver_trans_intra_6pc.cpp'); toc
