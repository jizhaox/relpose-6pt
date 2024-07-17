// This file is adapted from PoseLib
// https://github.com/PoseLib/PoseLib

#pragma once

Eigen::Matrix3d quat_to_rotmat(const Eigen::Vector4d& q) {
    return Eigen::Quaterniond(q(0), q(1), q(2), q(3)).toRotationMatrix();
}
Eigen::Vector4d rotmat_to_quat(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q_flip(R);
    Eigen::Vector4d q;
    q << q_flip.w(), q_flip.x(), q_flip.y(), q_flip.z();
    q.normalize();
    return q;
}
Eigen::Vector3d quat_rotate(const Eigen::Vector4d& q, const Eigen::Vector3d& p) {
    const double q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3);
    const double p1 = p(0), p2 = p(1), p3 = p(2);
    const double px1 = -p1 * q2 - p2 * q3 - p3 * q4;
    const double px2 = p1 * q1 - p2 * q4 + p3 * q3;
    const double px3 = p2 * q1 + p1 * q4 - p3 * q2;
    const double px4 = p2 * q2 - p1 * q3 + p3 * q1;
    return Eigen::Vector3d(px2 * q1 - px1 * q2 - px3 * q4 + px4 * q3, px3 * q1 - px1 * q3 + px2 * q4 - px4 * q2,
        px3 * q2 - px2 * q3 - px1 * q4 + px4 * q1);
}
Eigen::Vector4d quat_conj(const Eigen::Vector4d& q) { return Eigen::Vector4d(q(0), -q(1), -q(2), -q(3)); }

Eigen::Vector4d quat_multiply(const Eigen::Vector4d& qa, const Eigen::Vector4d& qb) {
    const double qa1 = qa(0), qa2 = qa(1), qa3 = qa(2), qa4 = qa(3);
    const double qb1 = qb(0), qb2 = qb(1), qb3 = qb(2), qb4 = qb(3);

    return Eigen::Vector4d(qa1 * qb1 - qa2 * qb2 - qa3 * qb3 - qa4 * qb4, qa1 * qb2 + qa2 * qb1 + qa3 * qb4 - qa4 * qb3,
        qa1 * qb3 + qa3 * qb1 - qa2 * qb4 + qa4 * qb2,
        qa1 * qb4 + qa2 * qb3 - qa3 * qb2 + qa4 * qb1);
}

void rotm2cayley(Eigen::Matrix<double, 3, 3>& rotm, Eigen::Matrix<double, 3, 1>& q)
{
    Eigen::Matrix3d eyeM = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix3d C1 = rotm - eyeM;
    Eigen::Matrix3d C2 = rotm + eyeM;
    Eigen::Matrix3d C = C1 * C2.inverse();
    q(0, 0) = -C(1, 2);
    q(1, 0) = C(0, 2);
    q(2, 0) = -C(0, 1);
}

void rotm2cayley(std::vector<Eigen::Matrix<double, 3, 3>>& rotm, std::vector<Eigen::Matrix<double, 3, 1>>& q)
{
    q.clear();
    Eigen::Matrix3d eyeM = Eigen::Matrix3d::Identity(3, 3);
    for (int i = 0; i < rotm.size(); i++)
    {
        Eigen::Matrix3d R = rotm[i];
        Eigen::Matrix3d C1 = R - eyeM;
        Eigen::Matrix3d C2 = R + eyeM;
        Eigen::Matrix3d C = C1 * C2.inverse();
        Eigen::Matrix<double, 3, 1> q0(-C(1, 2), C(0, 2), -C(0, 1));
        q.push_back(q0);
    }
}

Eigen::Vector4d quat_exp(const Eigen::Vector3d& w) {
    const double theta2 = w.squaredNorm();
    const double theta = std::sqrt(theta2);
    const double theta_half = 0.5 * theta;

    double re, im;
    if (theta > 1e-6) {
        re = std::cos(theta_half);
        im = std::sin(theta_half) / theta;
    }
    else {
        // we are close to zero, use taylor expansion to avoid problems
        // with zero divisors in sin(theta/2)/theta
        const double theta4 = theta2 * theta2;
        re = 1.0 - (1.0 / 8.0) * theta2 + (1.0 / 384.0) * theta4;
        im = 0.5 - (1.0 / 48.0) * theta2 + (1.0 / 3840.0) * theta4;

        // for the linearized part we re-normalize to ensure unit length
        // here s should be roughly 1.0 anyways, so no problem with zero div
        const double s = std::sqrt(re * re + im * im * theta2);
        re /= s;
        im /= s;
    }
    return Eigen::Vector4d(re, im * w(0), im * w(1), im * w(2));
}

Eigen::Vector4d quat_step_pre(const Eigen::Vector4d& q, const Eigen::Vector3d& w_delta) {
    return quat_multiply(quat_exp(w_delta), q);
}

struct CameraPose {
    // Rotation is represented as a unit quaternion
    // with real part first, i.e. QW, QX, QY, QZ
    Eigen::Vector4d q;
    Eigen::Vector3d t;

    // Constructors (Defaults to identity camera)
    CameraPose() : q(1.0, 0.0, 0.0, 0.0), t(0.0, 0.0, 0.0) {}
    CameraPose(const Eigen::Vector4d& qq, const Eigen::Vector3d& tt) : q(qq), t(tt) {}
    CameraPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& tt) : q(rotmat_to_quat(R)), t(tt) {}

    // Helper functions
    inline Eigen::Matrix3d R() const { return quat_to_rotmat(q); }
    inline Eigen::Matrix<double, 3, 4> Rt() const {
        Eigen::Matrix<double, 3, 4> tmp;
        tmp.block<3, 3>(0, 0) = quat_to_rotmat(q);
        tmp.col(3) = t;
        return tmp;
    }
    inline Eigen::Vector3d rotate(const Eigen::Vector3d& p) const { return quat_rotate(q, p); }
    inline Eigen::Vector3d derotate(const Eigen::Vector3d& p) const { return quat_rotate(quat_conj(q), p); }
    inline Eigen::Vector3d apply(const Eigen::Vector3d& p) const { return rotate(p) + t; }

    inline Eigen::Vector3d center() const { return -derotate(t); }
};

void root_refinement(const std::vector<Eigen::Vector3d>& p1, const std::vector<Eigen::Vector3d>& x1,
    const std::vector<Eigen::Vector3d>& p2, const std::vector<Eigen::Vector3d>& x2,
    std::vector<CameraPose>* output) {

    Eigen::Matrix<double, 6, 6> J;
    Eigen::Matrix<double, 6, 1> res;
    Eigen::Matrix<double, 6, 1> dp;
    Eigen::Matrix<double, 3, 3> sw;
    sw.setZero();

    std::vector<Eigen::Vector3d> qq1(6), qq2(6);
    for (size_t pt_k = 0; pt_k < 6; ++pt_k) {
        qq1[pt_k] = x1[pt_k].cross(p1[pt_k]);
        qq2[pt_k] = x2[pt_k].cross(p2[pt_k]);
    }

    for (size_t pose_k = 0; pose_k < output->size(); ++pose_k) {
        CameraPose& pose = (*output)[pose_k];

        for (size_t iter = 0; iter < 5; ++iter) {

            // compute residual and jacobian
            for (size_t pt_k = 0; pt_k < 6; ++pt_k) {
                Eigen::Vector3d x2t = x2[pt_k].cross(pose.t);
                Eigen::Vector3d Rx1 = pose.rotate(x1[pt_k]);
                Eigen::Vector3d Rqq1 = pose.rotate(qq1[pt_k]);

                res(pt_k) = (x2t - qq2[pt_k]).dot(Rx1) - x2[pt_k].dot(Rqq1);
                J.block<1, 3>(pt_k, 0) = -x2t.cross(Rx1) + qq2[pt_k].cross(Rx1) + x2[pt_k].cross(Rqq1);
                J.block<1, 3>(pt_k, 3) = -x2[pt_k].cross(Rx1);
            }

            if (res.norm() < 1e-12) {
                break;
            }

            dp = J.partialPivLu().solve(res);

            Eigen::Vector3d w = -dp.block<3, 1>(0, 0);
            pose.q = quat_step_pre(pose.q, w);
            pose.t = pose.t - dp.block<3, 1>(3, 0);
        }
    }
}

bool check_cheirality(const CameraPose& pose, const Eigen::Vector3d& p1, const Eigen::Vector3d& x1,
    const Eigen::Vector3d& p2, const Eigen::Vector3d& x2, double min_depth = 0.0) {

    // This code assumes that x1 and x2 are unit vectors
    const Eigen::Vector3d Rx1 = pose.rotate(x1);

    // [1 a; a 1] * [lambda1; lambda2] = [b1; b2]
    // [lambda1; lambda2] = [1 -a; -a 1] * [b1; b2] / (1 - a*a)
    const Eigen::Vector3d rhs = pose.t + pose.rotate(p1) - p2;
    const double a = -Rx1.dot(x2);
    const double b1 = -Rx1.dot(rhs);
    const double b2 = x2.dot(rhs);

    // Note that we drop the factor 1.0/(1-a*a) since it is always positive.
    const double lambda1 = b1 - a * b2;
    const double lambda2 = -a * b1 + b2;

    min_depth = min_depth * (1 - a * a);
    return lambda1 > min_depth && lambda2 > min_depth;
}

void cheirality_and_refinement_poselib(std::vector<Eigen::Matrix<double, 3, 3>> &Rs,
    std::vector<Eigen::Matrix<double, 3, 1>> &ts,
    std::vector<Eigen::Matrix<double, 3, 1>>& qs,
    double* Image_1, double* Image_2,
    double* extrinsic_R_camera, double* extrinsic_T_camera,
    PC_TYPE pctype)
{
    std::vector<Eigen::Matrix3d> R_camera;
    std::vector<Eigen::Vector3d> T_camera, Image1, Image2;
    std::vector<Eigen::Vector3d> p1, p2, x1, x2;

    int n_roots = Rs.size();
    vector<CameraPose> Rts_out;
    //Rts_out->reserve(n_roots);

    if (pctype == PC_TYPE::PC_GENERIC_CAM_CONSTRAINT)
    {
        pc_format_convert_generic(Image_1, Image_2, extrinsic_R_camera, extrinsic_T_camera,
            Image1, Image2, R_camera, T_camera);

        p1.push_back(T_camera[0]);
        x1.push_back(R_camera[0] * Image1[0]);
        p2.push_back(T_camera[1]);
        x2.push_back(R_camera[1] * Image2[0]);

        p1.push_back(T_camera[2]);
        x1.push_back(R_camera[2] * Image1[1]);
        p2.push_back(T_camera[3]);
        x2.push_back(R_camera[3] * Image2[1]);

        p1.push_back(T_camera[4]);
        x1.push_back(R_camera[4] * Image1[2]);
        p2.push_back(T_camera[5]);
        x2.push_back(R_camera[5] * Image2[2]);

        p1.push_back(T_camera[6]);
        x1.push_back(R_camera[6] * Image1[3]);
        p2.push_back(T_camera[7]);
        x2.push_back(R_camera[7] * Image2[3]);

        p1.push_back(T_camera[8]);
        x1.push_back(R_camera[8] * Image1[4]);
        p2.push_back(T_camera[9]);
        x2.push_back(R_camera[9] * Image2[4]);

        p1.push_back(T_camera[10]);
        x1.push_back(R_camera[10] * Image1[5]);
        p2.push_back(T_camera[11]);
        x2.push_back(R_camera[11] * Image2[5]);
    }
    else if (pctype == PC_TYPE::PC_INTRA_CAM_CONSTRAINT_FULL)
    {
        format_convert_pc(Image_1, Image_2, extrinsic_R_camera, extrinsic_T_camera,
            Image1, Image2, R_camera, T_camera);
        for (int i = 0; i < 6; i++)
        {
            if (i < 3)
            {
                p1.push_back(T_camera[0]);
                x1.push_back(R_camera[0] * Image1[i]);
                p2.push_back(T_camera[0]);
                x2.push_back(R_camera[0] * Image2[i]);
            }
            else
            {
                p1.push_back(T_camera[1]);
                x1.push_back(R_camera[1] * Image1[i]);
                p2.push_back(T_camera[1]);
                x2.push_back(R_camera[1] * Image2[i]);
            }
        }
        //  check_input(p1, x1, p2, x2, R_gt, t_gt);
    }
    else if (pctype == PC_TYPE::PC_INTER_CAM_CONSTRAINT_FULL || pctype == PC_TYPE::PC_INTER_CAM_CONSTRAINT_PARTIAL)
    {
        format_convert_pc(Image_1, Image_2, extrinsic_R_camera, extrinsic_T_camera,
            Image1, Image2, R_camera, T_camera);

        for (int i = 0; i < 6; i++)
        {
            if (i < 3)
            {
                p1.push_back(T_camera[0]);
                x1.push_back(R_camera[0] * Image1[i]);
                p2.push_back(T_camera[1]);
                x2.push_back(R_camera[1] * Image2[i]);
            }
            else
            {
                p1.push_back(T_camera[1]);
                x1.push_back(R_camera[1] * Image1[i]);
                p2.push_back(T_camera[0]);
                x2.push_back(R_camera[0] * Image2[i]);
            }
        }
        //   check_input(p1, x1, p2, x2, R_gt, t_gt);
    }

    for (int sol_k = 0; sol_k < n_roots; ++sol_k) {
        CameraPose pose;
        Eigen::Matrix<double, 3, 3> R = Rs[sol_k];
        Eigen::Quaterniond q(R);
        pose.q << q.w(), q.x(), q.y(), q.z();
        pose.t = ts[sol_k];

        bool cheiral_ok = true;
        for (size_t pt_k = 0; pt_k < 6; ++pt_k) {
            if (!check_cheirality(pose, p1[pt_k], x1[pt_k], p2[pt_k], x2[pt_k])) {
                cheiral_ok = false;
                break;
            }
        }
        if (!cheiral_ok) {
            continue;
        }
        Rts_out.push_back(pose);
    }
    root_refinement(p1, x1, p2, x2, &Rts_out);

    Rs.clear();
    ts.clear();
    qs.clear();
    
    for (std::vector<CameraPose>::iterator iter = Rts_out.begin(); iter != Rts_out.end(); iter++)
    {
        Eigen::Matrix3d itR = iter->R();
        Eigen::Matrix<double, 3, 1> itq;
        rotm2cayley(itR, itq);
        Rs.push_back(itR);
        ts.push_back(iter->t);
        qs.push_back(itq);
    }
}
