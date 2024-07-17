#include "mex.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "common.h"
#include "solver_trans_generic_6pc_core.h"
#include "cheirality_and_refinement.h"

using namespace std;
using namespace Eigen;

Eigen::MatrixXcd solve_equation_generic_cam(double *input, double* zr, double* zi)
{
    const VectorXd data = Map<const VectorXd>(input, 84*15);
    Eigen::MatrixXcd sols = solver_trans_generic_6pc_core(data);
    for (Index i = 0; i < sols.size(); i++) {
        zr[i] = sols(i).real();
        zi[i] = sols(i).imag();
    }
    return sols;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
    if (nrhs < 4)
        mexErrMsgTxt("at least 4 inputs are required!");
    if (nlhs < 1)
        mexErrMsgTxt("at least 1 output is required!");
    
    if (mxIsEmpty(prhs[0]) || mxIsEmpty(prhs[1]) || mxIsEmpty(prhs[2]) || mxIsEmpty(prhs[3]))
        mexErrMsgTxt("input parameter should not be an empty array!");

    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) 
        || !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1])
        || !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2])
        || !mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]))
    {
        mexErrMsgIdAndTxt("JPL:tr_generic_6pc:notDouble", "Input data must be type double.");
    }

    bool is_known_angle = false;
    PC_TYPE pctype = PC_TYPE::PC_GENERIC_CAM_CONSTRAINT;

    double *Image_1;
    double *Image_2;
    double *extrinsic_R_camera;
    double *extrinsic_T_camera;

    Image_1 = (double *)mxGetData(prhs[0]);
    Image_2 = (double *)mxGetData(prhs[1]);
    extrinsic_R_camera = (double *)mxGetData(prhs[2]);
    extrinsic_T_camera = (double *)mxGetData(prhs[3]);

    double *coeffs, *input;
    plhs[3] = mxCreateDoubleMatrix(3, 64, mxCOMPLEX);
    plhs[4] = mxCreateDoubleMatrix(15, 84, mxREAL);
    input = new double[84*15];

    double* zr = mxGetPr(plhs[3]);
    double* zi = mxGetPi(plhs[3]);
    coeffs = mxGetPr(plhs[4]);
    std::vector<std::vector<Eigen::Matrix<double,1,10>>> M;
    create_coeffs_pc(coeffs, input, M, Image_1, Image_2, extrinsic_R_camera, extrinsic_T_camera, pctype, is_known_angle);
    Eigen::MatrixXcd sols;
    sols = solve_equation_generic_cam(input, zr, zi);

    std::vector<Eigen::Matrix<double,3,1>> q_arr, t_arr;
    std::vector<Eigen::Matrix<double,3,3>> rotm;
    calculate_translation(sols, M, q_arr, t_arr, is_known_angle);
    cayley2rotm(rotm, q_arr);
    cheirality_and_refinement_poselib(rotm, t_arr, q_arr, Image_1, Image_2, extrinsic_R_camera, extrinsic_T_camera, pctype);
    int n_real_sol = q_arr.size();
    double *q_real_sols, *t_real_sols, *rotm_real_sols;
    if (n_real_sol > 0)
    {
        plhs[0] = mxCreateDoubleMatrix(3, n_real_sol, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(3, n_real_sol, mxREAL);

        mwSize dims[3] = {3,3,n_real_sol};
        plhs[2] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);

        q_real_sols = mxGetPr(plhs[0]);
        t_real_sols = mxGetPr(plhs[1]);
        rotm_real_sols = mxGetPr(plhs[2]);
        for (int i = 0; i < n_real_sol; i++)
        {
            Eigen::Matrix<double,3,1> q = q_arr[i];
            Eigen::Matrix<double,3,1> t = t_arr[i];
            Eigen::Matrix<double,3,3> r = rotm[i];
            q_real_sols[i*3] = q(0);
            q_real_sols[i*3+1] = q(1);
            q_real_sols[i*3+2] = q(2);
            t_real_sols[i*3] = t(0);
            t_real_sols[i*3+1] = t(1);
            t_real_sols[i*3+2] = t(2);

            rotm_real_sols[i*9] = r(0,0);
            rotm_real_sols[i*9+1] = r(1,0);
            rotm_real_sols[i*9+2] = r(2,0);
            rotm_real_sols[i*9+3] = r(0,1);
            rotm_real_sols[i*9+4] = r(1,1);
            rotm_real_sols[i*9+5] = r(2,1);
            rotm_real_sols[i*9+6] = r(0,2);
            rotm_real_sols[i*9+7] = r(1,2);
            rotm_real_sols[i*9+8] = r(2,2);
        }
    }
    else
    {
        plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
        plhs[2] = mxCreateDoubleMatrix(0, 0, mxREAL);
    }

    delete [] input;
    return;
}
