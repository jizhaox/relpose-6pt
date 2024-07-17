# README
This package implements relative pose estimation for multi-camera systems from six point correspondences, including a generic minimal solver for the generalized camera (`solver_trans_generic_6pc`) and two minimal solvers for popular configurations of two-camera rigs (`solver_trans_inter_6pc`, `solver_trans_intra_6pc`).

Source codes and Matlab mex files with demo code are provided in the package. The core solvers are written by C++. Matlab mex files are compiled using Ubuntu 16.04 + Matlab R2019a. Run `test_solver_pc.m` in folder "test".


# Reference

[1] [Banglei Guan](https://guanbanglei.github.io/), [Ji Zhao](https://sites.google.com/site/drjizhao), and [Laurent Kneip](https://mpl.sist.shanghaitech.edu.cn/). [**Six-Point Method for Multi-Camera Systems with Reduced Solution Space**](https://arxiv.org/pdf/2402.18066). European Conference on Computer Vision, 2024.

If you use this package in an academic work, please cite:

    @inproceedings{guan2024six,
      title={Six-Point Method for Multi-Camera Systems with Reduced Solution Space},
      author={Guan, Banglei and Zhao, Ji and Kneip, Laurent},
      booktitle={European Conference on Computer Vision},
      year={2024}
     }

The code was written by [Ji Zhao](https://sites.google.com/site/drjizhao). The initial version of this code was developed in 2020 and is available at [GitLab](https://gitlab.com/jizhaox/relpose-mcs/)

# solver_trans_generic_6pc

A generic minimal solver for the relative pose estimation of generalized camera using six point correspondences. Returns a maximum of 64 solutions.
* **Solver**:  `solver_trans_generic_6pc.mexa64`   

* **API**: `[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_generic_6pc(Image1, Image2, R_cam, t_cam);`

* **Input data for Demo**: 

     `Image1` (3\*6 matrix): normalized homogeneous image coordinates of six feature points expressed in view 1.

     `Image2` (3\*6 matrix): normalized homogeneous image coordinates of six feature points expressed in view 2.

     `R_cam` (3\*3\*12 matrix): extrinsic rotation of twelve virtual perspective cameras expressed in the reference of the multi-camera system.

     `t_cam` (3\*12 matrix): extrinsic translation of twelve virtual perspective cameras expressed in the reference of the multi-camera system.

* **Output data for Demo**: 

     `cay_sols` (3\*N matrix): real number solutions of Cayley rotation parameter, N is the number of real number solutions.

     `t_sols` (3\*N matrix): real number solutions of translation.

     `R_sols` (3\*3\*N matrix): real number solutions of rotation matrix.

     `cay_sols_all` (3\*M matrix): all solutions of Cayley rotation parameter, including real number solutions and complex number solutions. M is the number of all the solutions.


# solver_trans_inter_6pc

A minimal solver for the relative pose estimation of two-camera rigs using six inter-camera point correspondences. Returns a maximum of 48 solutions or 56 solutions.
* **Solver**:  `solver_trans_inter_6pc.mexa64`   

* **API**: `[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_inter_6pc(Image1, Image2, R_cam, t_cam, 0);` `[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_inter_6pc(Image1, Image2, R_cam, t_cam, 1);`

* **Input data for Demo**: 

     `Image1` (3\*6 matrix): normalized homogeneous image coordinates of six inter-camera feature points expressed in view 1.

     `Image2` (3\*6 matrix): normalized homogeneous image coordinates of six inter-camera feature points expressed in view 2.

     `R_cam` (3\*3\*2 matrix): extrinsic rotation of two cameras expressed in the reference of the multi-camera system.

     `t_cam` (3\*2 matrix): extrinsic translation of two cameras expressed in the reference of the multi-camera system.

     `Option`: 0 refers the `6pt-Our-inter48 solver`, 1 refers the `6pt-Our-inter56 solver`.

* **Output data for Demo**: 

     `cay_sols` (3\*N matrix): real number solutions of Cayley rotation parameter, N is the number of real number solutions.

     `t_sols` (3\*N matrix): real number solutions of translation.

     `R_sols` (3\*3\*N matrix): real number solutions of rotation matrix.

     `cay_sols_all` (3\*M matrix): all solutions of Cayley rotation parameter, including real number solutions and complex number solutions. M is the number of all the solutions.


# solver_trans_intra_6pc

A minimal solver for the relative pose estimation of two-camera rigs using six intra-camera point correspondences. Returns a maximum of 48 solutions.
* **Solver**:  `solver_trans_intra_6pc.mexa64`

* **API**: `[cay_sols, t_sols, R_sols, cay_sols_all] = solver_trans_intra_6pc(Image1, Image2, R_cam, t_cam);`

* **Input data for Demo**: 

     `Image1` (3\*6 matrix): normalized homogeneous image coordinates of six intra-camera feature points expressed in view 1.

     `Image2` (3\*6 matrix): normalized homogeneous image coordinates of six intra-camera feature points expressed in view 2.

     `R_cam` (3\*3\*2 matrix): extrinsic rotation of two cameras expressed in the reference of the multi-camera system.

     `t_cam` (3\*2 matrix): extrinsic translation of two cameras expressed in the reference of the multi-camera system.

* **Output data for Demo**: 

     `cay_sols` (3\*N matrix): real number solutions of Cayley rotation parameter, N is the number of real number solutions.

     `t_sols` (3\*N matrix): real number solutions of translation.

     `R_sols` (3\*3\*N matrix): real number solutions of rotation matrix.

     `cay_sols_all` (3\*M matrix): all solutions of Cayley rotation parameter, including real number solutions and complex number solutions. M is the number of all the solutions.


# Run

Compiled files using Ubuntu 16.04 + Matlab R2019a are provided. You can run the package in Matlab.

` test_solver_pc.m` is the demo which shows how to call the APIs.