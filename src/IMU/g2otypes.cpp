#include "g2otypes.h"

namespace g2o
{

using namespace ORB_SLAM2;

void EdgePRIDP::computeError()
{
    const Vector3d Pi = computePc();

    // err = obs - pi(Px)
    _error = _measurement - cam_project(Pi);
}

Vector3d EdgePRIDP::computePc()
{
    const VertexIDP* vIDP = static_cast<const VertexIDP*>(_vertices[0]);
    const VertexNavStatePR* vPR0 = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[2]);
    const VertexNavStatePR* vPRcb = static_cast<const VertexNavStatePR*>(_vertices[3]);

    //
    const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
    const Vector3d t0 = vPR0->estimate().Get_P();
    const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
    const Vector3d ti = vPRi->estimate().Get_P();
    const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
    const Vector3d tcb = vPRcb->estimate().Get_P();

    // point inverse depth in reference KF
    double rho = vIDP->estimate();
    if(rho<1e-6)
    {
        std::cerr<<"1. rho = "<<rho<<", rho<1e-6, shoudn't"<<std::endl;
        rho = 1e-6;
        setLevel(1);
    }
    // point coordinate in reference KF, body
    Vector3d P0;
    P0 << refnormxy[0], refnormxy[1], 1;
    double d=1.0/rho;   // depth
    P0 *= d;

    // Pi = Rcb*Ri^T*R0*Rcb^T* p0 + ( tcb - Rcb*Ri^T*R0*Rcb^T *tcb + Rcb*Ri^T*(t0-ti) )
    const Matrix3d Rcic0 = Rcb*Ri.transpose()*R0*Rcb.transpose();
    const Vector3d Pi = Rcic0*P0 + tcb - Rcic0*tcb + Rcb*Ri.transpose()*(t0-ti);
    return Pi;
}

void EdgePRIDP::linearizeOplus()
{
    const VertexIDP* vIDP = static_cast<const VertexIDP*>(_vertices[0]);
    const VertexNavStatePR* vPR0 = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[2]);
    const VertexNavStatePR* vPRcb = static_cast<const VertexNavStatePR*>(_vertices[3]);

    //
    const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
    const Vector3d t0 = vPR0->estimate().Get_P();
    const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
    const Matrix3d RiT = Ri.transpose();
    const Vector3d ti = vPRi->estimate().Get_P();
    const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
    const Vector3d tcb = vPRcb->estimate().Get_P();

    // point inverse depth in reference KF
    double rho = vIDP->estimate();
    if(rho<1e-6)
    {
        std::cerr<<"2. rho = "<<rho<<", rho<1e-6, shoudn't"<<std::endl;
        rho = 1e-6;
        setLevel(1);    //todo
    }
    // point coordinate in reference KF, body
    Vector3d P0;
    P0 << refnormxy[0], refnormxy[1], 1;
    double d=1.0/rho;   // depth
    P0 *= d;

    // Pi = Rcb*Ri^T*R0*Rcb^T* p0 + ( tcb - Rcb*Ri^T*R0*Rcb^T *tcb + Rcb*Ri^T*(t0-ti) )
    const Matrix3d Rcic0 = Rcb*Ri.transpose()*R0*Rcb.transpose();
    const Vector3d Pi = Rcic0*P0 + tcb - Rcic0*tcb + Rcb*Ri.transpose()*(t0-ti);

    // err = obs - pi(Px)
    // Jx = -Jpi * dpi/dx
    double x = Pi[0];
    double y = Pi[1];
    double z = Pi[2];
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // 1. J_e_rho, 2x1
    Vector3d J_pi_rho = Rcic0*(-d*P0);
    _jacobianOplus[0] = -Jpi * J_pi_rho;

    // 2. J_e_pr0, 2x6
    Matrix3d J_pi_t0 = Rcb*RiT;
    Matrix3d J_pi_r0 = -Rcic0*SO3::hat(P0-tcb)*Rcb;
    Matrix<double,3,6> J_pi_pr0;
    J_pi_pr0.topLeftCorner(3,3) = J_pi_t0;
    J_pi_pr0.topRightCorner(3,3) = J_pi_r0;
    _jacobianOplus[1] = -Jpi * J_pi_pr0;

    // 3. J_e_pri, 2x6
    Matrix3d J_pi_ti = -Rcb*RiT;
    Vector3d taux = RiT* ( R0*Rcb.transpose()*(P0 - tcb) + t0 - ti );
    Matrix3d J_pi_ri = Rcb*SO3::hat(taux);
    Matrix<double,3,6> J_pi_pri;
    J_pi_pri.topLeftCorner(3,3) = J_pi_ti;
    J_pi_pri.topRightCorner(3,3) = J_pi_ri;
    _jacobianOplus[2] = -Jpi * J_pi_pri;

    // 4. J_e_prcb, 2x6
    Matrix3d J_pi_tcb = Matrix3d::Identity() - Rcic0;
    Matrix3d J_pi_rcb = - SO3::hat( Rcic0*(P0-tcb) )*Rcb
                        + Rcic0* SO3::hat( P0-tcb )*Rcb
                        - Rcb* SO3::hat( RiT*(t0-ti) );
    Matrix<double,3,6> J_pi_prcb;
    J_pi_prcb.topLeftCorner(3,3) = J_pi_tcb;
    J_pi_prcb.topRightCorner(3,3) = J_pi_rcb;
    _jacobianOplus[3] = -Jpi * J_pi_prcb;
}

void EdgeNavStatePRV::computeError()
{
    //
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[0]);
    const VertexNavStatePR* vPRj = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStateV* vVi = static_cast<const VertexNavStateV*>(_vertices[2]);
    const VertexNavStateV* vVj = static_cast<const VertexNavStateV*>(_vertices[3]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[4]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.Get_P();
    const Sophus::SO3 Ri = NSPRi.Get_R();

    const NavState& NSVi = vVi->estimate();
    const Vector3d Vi = NSVi.Get_V();
    // Bias from the bias vertex
    const NavState& NSBiasi = vBiasi->estimate();
    const Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    const Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.Get_P();
    const Sophus::SO3 Rj = NSPRj.Get_R();

    const NavState& NSVj = vVj->estimate();
    const Vector3d Vj = NSVj.Get_V();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    const double dTij = M.getDeltaTime();   // Delta Time
    const double dT2 = dTij*dTij;
    const Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    const Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    const Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());  // Delta Rotation pre-integration measurement

    // tmp variable, transpose of Ri
    const Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    const Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                          - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    const Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                          - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    const Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    const Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    const Vector3d rPhiij = rRij.log();


    Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
    err.setZero();

    // 9-Dim error vector order:
    // position-velocity-rotation
    // rPij - rPhiij - rVij
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rPhiij;     // rotation phi error
    err.segment<3>(6) = rVij;       // velocity error

    _error = err;
}

void EdgeNavStatePRV::linearizeOplus()
{
    //
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[0]);
    const VertexNavStatePR* vPRj = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStateV* vVi = static_cast<const VertexNavStateV*>(_vertices[2]);
    const VertexNavStateV* vVj = static_cast<const VertexNavStateV*>(_vertices[3]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[4]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.Get_P();
    const Matrix3d Ri = NSPRi.Get_RotMatrix();

    const NavState& NSVi = vVi->estimate();
    const Vector3d Vi = NSVi.Get_V();
    // Bias from the bias vertex
    const NavState& NSBiasi = vBiasi->estimate();
    const Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    //const Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.Get_P();
    const Matrix3d Rj = NSPRj.Get_RotMatrix();

    const NavState& NSVj = vVj->estimate();
    const Vector3d Vj = NSVj.Get_V();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    const double dTij = M.getDeltaTime();   // Delta Time
    const double dT2 = dTij*dTij;

    // some temp variable
    //Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
    Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
    Matrix3d RiT = Ri.transpose();          // Ri^T
    Matrix3d RjT = Rj.transpose();          // Rj^T
    Vector3d rPhiij = _error.segment<3>(3); // residual of rotation, rPhiij
    Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
    Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

    // 1.
    // increment is the same as Forster 15'RSS
    // pi = pi + dpi,    pj = pj + dpj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    // vi = vi + dvi,       vj = vj + dvj
    //      Note: the optimized bias term is the 'delta bias'
    // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
    // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

    // 2.
    // 9-Dim error vector order in PVR:
    // position-velocity-rotation
    // rPij - rPhiij - rVij
    //      Jacobian row order:
    // J_rPij_xxx
    // J_rPhiij_xxx
    // J_rVij_xxx

    // 3.
    // order in 'update_' in PR
    // Vertex_i : dPi, dPhi_i
    // Vertex_j : dPj, dPhi_j
    // 6-Dim error vector order in Bias:
    // dBiasg_i - dBiasa_i

    // Jacobians:
    // dP/dPR0, dP/dPR1, dP/dV0, dP/dV1, dP/dBias0
    // dR/dPR0, dR/dPR1, dR/dV0, dR/dV1, dR/dBias0
    // dV/dPR0, dV/dPR1, dV/dV0, dV/dV1, dV/dBias0

    // 4. PR0 & V0
    // For Vertex_PR_i, J [dP;dR;dV] / [dP0 dR0]
    Matrix<double,9,6> JPRi;
    JPRi.setZero();
    // J_rPij_xxx_i for Vertex_PR_i
    JPRi.block<3,3>(0,0) = - RiT;      //J_rP_dpi
    JPRi.block<3,3>(0,3) = Sophus::SO3::hat( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i
    // J_rPhiij_xxx_i for Vertex_PR_i
    Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
    Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg*dBgi);
    JPRi.block<3,3>(3,0) = O3x3;    //dpi
    JPRi.block<3,3>(3,3) = - JrInv_rPhi * RjT * Ri;    //dphi_i
    // J_rVij_xxx_i for Vertex_PVR_i
    JPRi.block<3,3>(6,0) = O3x3;    //dpi
    JPRi.block<3,3>(6,3) = Sophus::SO3::hat( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i

    // For Vertex_V_i, J [dP;dR;dV] / dV0
    Matrix<double,9,3> JVi;
    JVi.setZero();
    JVi.block<3,3>(0,0) = - RiT*dTij;  //J_rP_dvi
    JVi.block<3,3>(3,0) = O3x3;    //rR_dvi
    JVi.block<3,3>(6,0) = - RiT;    //rV_dvi

    // 5. PR1 & V1
    // For Vertex_PR_j, J [dP;dR;dV] / [dP1 dR1]
    Matrix<double,9,6> JPRj;
    JPRj.setZero();
    // J_rPij_xxx_j for Vertex_PR_j
    JPRj.block<3,3>(0,0) = RiT;  //rP_dpj
    JPRj.block<3,3>(0,3) = O3x3;    //rP_dphi_j
    // J_rPhiij_xxx_j for Vertex_PR_j
    JPRj.block<3,3>(3,0) = O3x3;    //rR_dpj
    JPRj.block<3,3>(3,3) = JrInv_rPhi;    //rR_dphi_j
    // J_rVij_xxx_j for Vertex_PR_j
    JPRj.block<3,3>(6,0) = O3x3;    //rV_dpj
    JPRj.block<3,3>(6,3) = O3x3;    //rV_dphi_j

    // For Vertex_V_i, J [dP;dR;dV] / dV1
    Matrix<double,9,3> JVj;
    JVj.setZero();
    JVj.block<3,3>(0,0) = O3x3;    //rP_dvj
    JVj.block<3,3>(3,0) = O3x3;    //rR_dvj
    JVj.block<3,3>(6,0) = RiT;    //rV_dvj

    // 6.
    // For Vertex_Bias_i
    Matrix<double,9,6> JBiasi;
    JBiasi.setZero();

    // 5.1
    // J_rPij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(0,0) = - M.getJPBiasg();     //J_rP_dbgi
    JBiasi.block<3,3>(0,3) = - M.getJPBiasa();     //J_rP_dbai

    // J_rPhiij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(3,0) = - JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
    JBiasi.block<3,3>(3,3) = O3x3;    //dba_i

    // J_rVij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(6,0) = - M.getJVBiasg();    //dbg_i
    JBiasi.block<3,3>(6,3) = - M.getJVBiasa();    //dba_i

    // Evaluate _jacobianOplus
    _jacobianOplus[0] = JPRi;
    _jacobianOplus[1] = JPRj;
    _jacobianOplus[2] = JVi;
    _jacobianOplus[3] = JVj;
    _jacobianOplus[4] = JBiasi;
}

void EdgeNavStatePRPointXYZ::linearizeOplus()
{
    const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const VertexNavStatePR* vNavState = static_cast<const VertexNavStatePR*>(_vertices[1]);

    const NavState& ns = vNavState->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of error w.r.t Pw
    _jacobianOplusXi = - Jpi * Rcb * Rwb.transpose();

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb*Rbw;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavState PR
    // order in 'update_': dP, dPhi
    Matrix<double,2,6> JNavState = Matrix<double,2,6>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    JNavState.block<2,3>(0,3) = JdRwb;

    // Jacobian of error w.r.t NavState
    _jacobianOplusXj = JNavState;
}

void EdgeNavStatePRPointXYZOnlyPose::linearizeOplus()
{
    const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);

    const NavState& ns = vNSPR->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    //const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb*Rbw;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavStatePR
    // order in 'update_': dP, dPhi
    Matrix<double,2,6> JNavState = Matrix<double,2,6>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    JNavState.block<2,3>(0,3) = JdRwb;

    // Jacobian of error w.r.t NavStatePR
    _jacobianOplusXi = JNavState;
}


void EdgeNavStatePriorPRVBias::computeError()
{
    const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);
    const VertexNavStateV* vNSV = static_cast<const VertexNavStateV*>(_vertices[1]);
    const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[2]);
    const NavState& nsPRest = vNSPR->estimate();
    const NavState& nsVest = vNSV->estimate();
    const NavState& nsBiasest = vNSBias->estimate();
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

    // PVR terms
    // eP = P_prior - P_est
    err.segment<3>(0) = nsprior.Get_P() - nsPRest.Get_P();
    // eR = log(R_prior^-1 * R_est)
    err.segment<3>(3) = (nsprior.Get_R().inverse() * nsPRest.Get_R()).log();
    // eV = V_prior - V_est
    err.segment<3>(6) = nsprior.Get_V() - nsVest.Get_V();

    // Bias terms
    // eB = Bias_prior - Bias_est
    // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
    // err_ba = (ba_prior+dba_prior) - (ba+dba)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());


    _error = err;

}

void EdgeNavStatePriorPRVBias::linearizeOplus()
{
    // Estimated NavState
    //const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);
    //const VertexNavStateV* vNSV = static_cast<const VertexNavStateV*>(_vertices[1]);
    //const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[2]);
    //const NavState& nsPRest = vNSPR->estimate();
    //const NavState& nsVest = vNSV->estimate();
    //const NavState& nsBiasest = vNSBias->estimate();
    //const NavState& nsprior = _measurement;

    Matrix<double,15,6> _jacobianOplusPR = Matrix<double,15,6>::Zero();
    _jacobianOplusPR.block<3,3>(0,0) = - Matrix3d::Identity();//nsPRest.Get_RotMatrix();
    _jacobianOplusPR.block<3,3>(3,3) = Sophus::SO3::JacobianRInv( _error.segment<3>(3) );
    Matrix<double,15,3> _jacobianOplusV = Matrix<double,15,3>::Zero();
    _jacobianOplusV.block<3,3>(6,6) = - Matrix3d::Identity();

//    Matrix<double,15,6> _jacobianOplusPR = Matrix<double,15,6>::Zero();
//    _jacobianOplusPR.block<3,3>(0,0) = ( nsprior.Get_R().inverse() * nsPRest.Get_R() ).matrix();
//    _jacobianOplusPR.block<3,3>(3,3) = Sophus::SO3::JacobianRInv( _error.segment<3>(3) );
//    Matrix<double,15,3> _jacobianOplusV = Matrix<double,15,3>::Zero();
//    _jacobianOplusV.block<3,3>(6,0) = Matrix3d::Identity();

    Matrix<double,15,6> _jacobianOplusBias = Matrix<double,15,6>::Zero();
    _jacobianOplusBias.block<3,3>(9,0) = - Matrix3d::Identity();
    _jacobianOplusBias.block<3,3>(12,3) = - Matrix3d::Identity();


    _jacobianOplus[0] = _jacobianOplusPR;
    _jacobianOplus[1] = _jacobianOplusV;
    _jacobianOplus[2] = _jacobianOplusBias;
}

void EdgeNavStatePVR::computeError()
{
    //
    const VertexNavStatePVR* vPVRi = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    const VertexNavStatePVR* vPVRj = static_cast<const VertexNavStatePVR*>(_vertices[1]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[2]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPVRi = vPVRi->estimate();
    Vector3d Pi = NSPVRi.Get_P();
    Vector3d Vi = NSPVRi.Get_V();
    Sophus::SO3 Ri = NSPVRi.Get_R();
    // Bias from the bias vertex
    const NavState& NSBiasi = vBiasi->estimate();
    Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPVRj = vPVRj->estimate();
    Vector3d Pj = NSPVRj.Get_P();
    Vector3d Vj = NSPVRj.Get_V();
    Sophus::SO3 Rj = NSPVRj.Get_R();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;
    Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());  // Delta Rotation pre-integration measurement

    // tmp variable, transpose of Ri
    Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                  - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                  - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    Vector3d rPhiij = rRij.log();


    Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
    err.setZero();

    // 9-Dim error vector order:
    // position-velocity-rotation
    // rPij - rVij - rPhiij
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rVij;       // velocity error
    err.segment<3>(6) = rPhiij;     // rotation phi error

    _error = err;

    //Test log
    if( (NSPVRi.Get_BiasGyr()-NSBiasi.Get_BiasGyr()).norm()>1e-6 || (NSPVRi.Get_BiasAcc()-NSBiasi.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"id pvri/pvrj/biasi: "<<vPVRi->id()<<"/"<<vPVRj->id()<<"/"<<vBiasi->id()<<std::endl;
        std::cerr<<"bias gyr not equal for PVR/Bias vertex"<<std::endl<<NSPVRi.Get_BiasGyr().transpose()<<" / "<<NSBiasi.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"bias acc not equal for PVR/Bias vertex"<<std::endl<<NSPVRi.Get_BiasAcc().transpose()<<" / "<<NSBiasi.Get_BiasAcc().transpose()<<std::endl;
    }

}

void EdgeNavStatePVR::linearizeOplus()
{
    //
    const VertexNavStatePVR* vPVRi = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    const VertexNavStatePVR* vPVRj = static_cast<const VertexNavStatePVR*>(_vertices[1]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[2]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPVRi = vPVRi->estimate();
    Vector3d Pi = NSPVRi.Get_P();
    Vector3d Vi = NSPVRi.Get_V();
    Matrix3d Ri = NSPVRi.Get_RotMatrix();
    // bias
    const NavState& NSBiasi = vBiasi->estimate();
    Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    //    Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPVRj = vPVRj->estimate();
    Vector3d Pj = NSPVRj.Get_P();
    Vector3d Vj = NSPVRj.Get_V();
    Matrix3d Rj = NSPVRj.Get_RotMatrix();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;

    // some temp variable
    //Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
    Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
    Matrix3d RiT = Ri.transpose();          // Ri^T
    Matrix3d RjT = Rj.transpose();          // Rj^T
    Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
    Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
    Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

    // 1.
    // increment is the same as Forster 15'RSS
    // pi = pi + dpi,    pj = pj + dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
    // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

    // 2.
    // 9-Dim error vector order in PVR:
    // position-velocity-rotation
    // rPij - rVij - rPhiij
    //      Jacobian row order:
    // J_rPij_xxx
    // J_rVij_xxx
    // J_rPhiij_xxx

    // 3.
    // order in 'update_' in PVR
    // Vertex_i : dPi, dVi, dPhi_i
    // Vertex_j : dPj, dVj, dPhi_j
    // 6-Dim error vector order in Bias:
    // dBiasg_i - dBiasa_i

    // 4.
    // For Vertex_PVR_i
    Matrix<double,9,9> JPVRi;
    JPVRi.setZero();

    // 4.1
    // J_rPij_xxx_i for Vertex_PVR_i
    JPVRi.block<3,3>(0,0) = - RiT;      //J_rP_dpi
    JPVRi.block<3,3>(0,3) = - RiT*dTij;  //J_rP_dvi
    //JPVRi.block<3,3>(0,6) = SO3Calc::skew( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i
    JPVRi.block<3,3>(0,6) = Sophus::SO3::hat( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i

    // 4.2
    // J_rVij_xxx_i for Vertex_PVR_i
    JPVRi.block<3,3>(3,0) = O3x3;    //dpi
    JPVRi.block<3,3>(3,3) = - RiT;    //dvi
    //JPVRi.block<3,3>(3,6) = SO3Calc::skew( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i
    JPVRi.block<3,3>(3,6) = Sophus::SO3::hat( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i

    // 4.3
    // J_rPhiij_xxx_i for Vertex_PVR_i
    //Matrix3d ExprPhiijTrans = SO3Calc::Expmap(rPhiij).transpose();  //Exp( rPhi )^T
    //Matrix3d JrBiasGCorr = SO3Calc::JacobianR(J_rPhi_dbg*dBgi);     //Jr( M.J_rPhi_bg * dBgi )
    Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
    Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg*dBgi);
    JPVRi.block<3,3>(6,0) = O3x3;    //dpi
    JPVRi.block<3,3>(6,3) = O3x3;    //dvi
    JPVRi.block<3,3>(6,6) = - JrInv_rPhi * RjT * Ri;    //dphi_i


    // 5.
    // For Vertex_PVR_j
    Matrix<double,9,9> JPVRj;
    JPVRj.setZero();

    // 5.1
    // J_rPij_xxx_j for Vertex_PVR_j
    JPVRj.block<3,3>(0,0) = RiT;  //dpj
    JPVRj.block<3,3>(0,3) = O3x3;    //dvj
    JPVRj.block<3,3>(0,6) = O3x3;    //dphi_j

    // 5.2
    // J_rVij_xxx_j for Vertex_PVR_j
    JPVRj.block<3,3>(3,0) = O3x3;    //dpj
    JPVRj.block<3,3>(3,3) = RiT;    //dvj
    JPVRj.block<3,3>(3,6) = O3x3;    //dphi_j

    // 5.3
    // J_rPhiij_xxx_j for Vertex_PVR_j
    JPVRj.block<3,3>(6,0) = O3x3;    //dpj
    JPVRj.block<3,3>(6,3) = O3x3;    //dvj
    JPVRj.block<3,3>(6,6) = JrInv_rPhi;    //dphi_j


    // 6.
    // For Vertex_Bias_i
    Matrix<double,9,6> JBiasi;
    JBiasi.setZero();

    // 5.1
    // J_rPij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(0,0) = - M.getJPBiasg();     //J_rP_dbgi
    JBiasi.block<3,3>(0,3) = - M.getJPBiasa();     //J_rP_dbai

    // J_rVij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(3,0) = - M.getJVBiasg();    //dbg_i
    JBiasi.block<3,3>(3,3) = - M.getJVBiasa();    //dba_i

    // J_rPhiij_xxx_j for Vertex_Bias_i
    JBiasi.block<3,3>(6,0) = - JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
    JBiasi.block<3,3>(6,3) = O3x3;    //dba_i

    // Evaluate _jacobianOplus
    _jacobianOplus[0] = JPVRi;
    _jacobianOplus[1] = JPVRj;
    _jacobianOplus[2] = JBiasi;
}

void EdgeNavStateBias::computeError()
{
    //
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[0]);
    const VertexNavStateBias* vBiasj = static_cast<const VertexNavStateBias*>(_vertices[1]);

    const NavState& NSi = vBiasi->estimate();
    const NavState& NSj = vBiasj->estimate();

    // residual error of Gyroscope's bias, Forster 15'RSS
    Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                    - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

    // residual error of Accelerometer's bias, Forster 15'RSS
    Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                    - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

    Vector6d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=6
    err.setZero();
    // 6-Dim error vector order:
    // deltabiasGyr_i-deltabiasAcc_i
    // rBiasGi - rBiasAi
    err.segment<3>(0) = rBiasG;     // bias gyro error
    err.segment<3>(3) = rBiasA;    // bias acc error

    _error = err;
}

void EdgeNavStateBias::linearizeOplus()
{
    // 6-Dim error vector order:
    // deltabiasGyr_i-deltabiasAcc_i
    // rBiasGi - rBiasAi

    _jacobianOplusXi = - Matrix<double,6,6>::Identity();
    _jacobianOplusXj = Matrix<double,6,6>::Identity();

}

void EdgeNavStatePVRPointXYZ::linearizeOplus()
{
    const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

    const NavState& ns = vNavState->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of error w.r.t Pw
    _jacobianOplusXi = - Jpi * Rcb * Rwb.transpose();

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavState
    // order in 'update_': dP, dV, dPhi
    Matrix<double,2,9> JNavState = Matrix<double,2,9>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    //JNavState.block<2,3>(0,3) = 0;
    JNavState.block<2,3>(0,6) = JdRwb;
    //JNavState.block<2,3>(0,9) = 0;
    //JNavState.block<2,3>(0,12) = 0;

    // Jacobian of error w.r.t NavState
    _jacobianOplusXj = JNavState;
}

void EdgeNavStatePVRPointXYZOnlyPose::linearizeOplus()
{
    const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);

    const NavState& ns = vNSPVR->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    //const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavStatePVR
    // order in 'update_': dP, dV, dPhi
    Matrix<double,2,9> JNavState = Matrix<double,2,9>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    //JNavState.block<2,3>(0,3) = 0;
    JNavState.block<2,3>(0,6) = JdRwb;
    //JNavState.block<2,3>(0,9) = 0;
    //JNavState.block<2,3>(0,12) = 0;

    // Jacobian of error w.r.t NavStatePVR
    _jacobianOplusXi = JNavState;
}

void EdgeNavStatePriorPVRBias::computeError()
{
    const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[1]);
    const NavState& nsPVRest = vNSPVR->estimate();
    const NavState& nsBiasest = vNSBias->estimate();
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

    // PVR terms
    // eP = P_prior - P_est
    err.segment<3>(0) = nsprior.Get_P() - nsPVRest.Get_P();
    // eV = V_prior - V_est
    err.segment<3>(3) = nsprior.Get_V() - nsPVRest.Get_V();
    // eR = log(R_prior^-1 * R_est)
    err.segment<3>(6) = (nsprior.Get_R().inverse() * nsPVRest.Get_R()).log();

    // Bias terms
    // eB = Bias_prior - Bias_est
    // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
    // err_ba = (ba_prior+dba_prior) - (ba+dba)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

    _error = err;

    //Debug log
    //std::cout<<"prior edge error: "<<std::endl<<_error.transpose()<<std::endl;

    //Test log
    if( (nsPVRest.Get_BiasGyr()-nsBiasest.Get_BiasGyr()).norm()>1e-6 || (nsPVRest.Get_BiasAcc()-nsBiasest.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"bias gyr not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias"<<std::endl<<nsPVRest.Get_BiasGyr().transpose()<<" / "<<nsBiasest.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"bias acc not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias"<<std::endl<<nsPVRest.Get_BiasAcc().transpose()<<" / "<<nsBiasest.Get_BiasAcc().transpose()<<std::endl;
    }
}

void EdgeNavStatePriorPVRBias::linearizeOplus()
{
    // Estimated NavState
    //const VertexNavStatePVR* vPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    //const VertexNavStateBias* vBias = static_cast<const VertexNavStateBias*>(_vertices[1]);
    //const NavState& nsPVRest = vPVR->estimate();

    _jacobianOplusXi = Matrix<double,15,9>::Zero();
    _jacobianOplusXi.block<3,3>(0,0) = - Matrix3d::Identity();//nsPVRest.Get_RotMatrix();
    _jacobianOplusXi.block<3,3>(3,3) = - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(6,6) = Sophus::SO3::JacobianRInv( _error.segment<3>(6) );

    _jacobianOplusXj = Matrix<double,15,6>::Zero();
    _jacobianOplusXj.block<3,3>(9,0) = - Matrix3d::Identity();
    _jacobianOplusXj.block<3,3>(12,3) = - Matrix3d::Identity();

}

//--------------------------------------

/**
 * @brief EdgeNavStatePrior::EdgeNavStatePrior
 */
void EdgeNavStatePrior::computeError()
{
    // Estimated NavState
    const VertexNavState* v = static_cast<const VertexNavState*>(_vertices[0]);
    const NavState& nsest = v->estimate();
    // Measurement: NavState_prior
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

//    // err_P = P - P_prior
//    err.segment<3>(0) = nsest.Get_P() - nsprior.Get_P();
//    // err_V = V - V_prior
//    err.segment<3>(3) = nsest.Get_V() - nsprior.Get_V();
//    // err_R = log (R * R_prior^-1)
//    err.segment<3>(6) = (nsest.Get_R() * nsprior.Get_R().inverse()).log();
//    // err_bg = (bg+dbg) - (bg_prior+dbg_prior)
//    err.segment<3>(9) = (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr()) - (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr());
//    // err_ba = (ba+dba) - (ba_prior+dba_prior)
//    err.segment<3>(12) = (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc()) - (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc());

    // err_P = P - P_prior
    err.segment<3>(0) = nsprior.Get_P() - nsest.Get_P();
    // err_V = V - V_prior
    err.segment<3>(3) = nsprior.Get_V() - nsest.Get_V();
    // err_R = log (R * R_prior^-1)
    err.segment<3>(6) = (nsprior.Get_R().inverse() * nsest.Get_R()).log();
    // err_bg = (bg+dbg) - (bg_prior+dbg_prior)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr());
    // err_ba = (ba+dba) - (ba_prior+dba_prior)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc());

    _error = err;

    //Debug log
    //std::cout<<"prior edge error: "<<std::endl<<_error.transpose()<<std::endl;
}

void EdgeNavStatePrior::linearizeOplus()
{
    // 1.
    // increment is the same as Forster 15'RSS
    // pi = pi + dpi
    // vi = vi + dvi
    // Ri = Ri*Exp(dphi_i)
    //      Note: the optimized bias term is the 'delta bias'
    // dBgi = dBgi + dbgi_update
    // dBai = dBai + dbai_update

    // Estimated NavState
    //const VertexNavState* v = static_cast<const VertexNavState*>(_vertices[0]);
    //const NavState& nsest = v->estimate();

//    _jacobianOplusXi.block<3,3>(0,0) = Matrix3d::Identity();//nsest.Get_RotMatrix();
//    _jacobianOplusXi.block<3,3>(3,3) = Matrix3d::Identity();
//    _jacobianOplusXi.block<3,3>(6,6) = Sophus::SO3::JacobianLInv( _error.segment<3>(6) ) * nsest.Get_RotMatrix();
//    _jacobianOplusXi.block<3,3>(9,9) = Matrix3d::Identity();
//    _jacobianOplusXi.block<3,3>(12,12) = Matrix3d::Identity();

    _jacobianOplusXi.block<3,3>(0,0) = - Matrix3d::Identity();//nsest.Get_RotMatrix();
    _jacobianOplusXi.block<3,3>(3,3) = - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(6,6) = Sophus::SO3::JacobianRInv( _error.segment<3>(6) );
    _jacobianOplusXi.block<3,3>(9,9) = - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(12,12) = - Matrix3d::Identity();

    //Debug log
    //std::cout<<"prior edge jacobian: "<<std::endl<<_jacobianOplusXi<<std::endl;
}

/**
 * @brief VertexNavState::VertexNavState
 */
VertexNavState::VertexNavState() : BaseVertex<15, NavState>()
{
}
// Todo
bool VertexNavState::read(std::istream& is) {return true;}
bool VertexNavState::write(std::ostream& os) const {return true;}

void VertexNavState::oplusImpl(const double* update_)
{
    // 1.
    // order in 'update_'
    // dP, dV, dPhi, dBiasGyr, dBiasAcc

    // 2.
    // the same as Forster 15'RSS
    // pi = pi + dpi,    pj = pj + dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
    // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

    Eigen::Map<const Vector15d> update(update_);
    _estimate.IncSmall(update);

    //std::cout<<"id "<<id()<<" ns update: "<<update.transpose()<<std::endl;
}

/**
 * @brief EdgeNavState::EdgeNavState
 */
EdgeNavState::EdgeNavState() : BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>()
{
}
// Todo
bool EdgeNavState::read(std::istream& is) {return true;}
bool EdgeNavState::write(std::ostream& os) const {return true;}

/**
 * @brief EdgeNavState::computeError
 * In g2o, computeError() is called in computeActiveErrors(), before buildSystem()
 */
void EdgeNavState::computeError()
{
    const VertexNavState* vi = static_cast<const VertexNavState*>(_vertices[0]);
    const VertexNavState* vj = static_cast<const VertexNavState*>(_vertices[1]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSi = vi->estimate();
    Vector3d Pi = NSi.Get_P();
    Vector3d Vi = NSi.Get_V();
    //Matrix3d Ri = NSi.Get_RotMatrix();
    Sophus::SO3 Ri = NSi.Get_R();
    Vector3d dBgi = NSi.Get_dBias_Gyr();
    Vector3d dBai = NSi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSj = vj->estimate();
    Vector3d Pj = NSj.Get_P();
    Vector3d Vj = NSj.Get_V();
    //Matrix3d Rj = NSj.Get_RotMatrix();
    Sophus::SO3 Rj = NSj.Get_R();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;
    Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    //Matrix3d dRij = M.getDeltaR();    // Delta Rotation pre-integration measurement
    Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());

    // tmp variable, transpose of Ri
    //Matrix3d RiT = Ri.transpose();
    Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                  - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                  - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    //Matrix3d dR_dbg = Sophus::SO3::exp(M.Get_J_Phi_Biasg()*dBgi).matrix();
    //Matrix3d rRij = (dRij * dR_dbg).transpose() * Ri.transpose()*Rj;
    //Vector3d rPhiij = Sophus::SO3(rRij).log();
    Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    Vector3d rPhiij = rRij.log();

    // residual error of Gyroscope's bias, Forster 15'RSS
    Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                    - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

    // residual error of Accelerometer's bias, Forster 15'RSS
    Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                    - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

    Vector15d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=15
    err.setZero();
    // 15-Dim error vector order:
    // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
    // rPij - rVij - rPhiij - rBiasGi - rBiasAi
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rVij;       // velocity error
    err.segment<3>(6) = rPhiij;     // rotation phi error
    err.segment<3>(9) = rBiasG;     // bias gyro error
    err.segment<3>(12) = rBiasA;    // bias acc error

    _error = err;

    // Debug log
    //std::cout<<"ns err: "<<_error.transpose()<<std::endl;
}

/**
 * @brief EdgeNavState::linearizeOplus
 * In g2o, linearizeOplus() is called in buildSystem(), after computeError()
 * So variable #_error# is computed and can be used here
 *
 * Reference:
 * [1]  Supplementary Material to :
 *      IMU Preintegration on Manifold for E cient Visual-Inertial Maximum-a-Posteriori Estimation IMU Preintegration
 *      Technical Report GT-IRIM-CP&R-2015-001
 *      Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza
 *      May 30, 2015
 * [2]  IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation
 *      Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza
 *      RSS, 2015
 */
void EdgeNavState::linearizeOplus()
{
    const VertexNavState* vi = static_cast<const VertexNavState*>(_vertices[0]);
    const VertexNavState* vj = static_cast<const VertexNavState*>(_vertices[1]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSi = vi->estimate();
    Vector3d Pi = NSi.Get_P();
    Vector3d Vi = NSi.Get_V();
    Matrix3d Ri = NSi.Get_RotMatrix();
    Vector3d dBgi = NSi.Get_dBias_Gyr();
    //    Vector3d dBai = NSi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSj = vj->estimate();
    Vector3d Pj = NSj.Get_P();
    Vector3d Vj = NSj.Get_V();
    Matrix3d Rj = NSj.Get_RotMatrix();

    // IMU Preintegration measurement
    const IMUPreintegrator& M =_measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;

    // some temp variable
    Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
    Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
    Matrix3d RiT = Ri.transpose();          // Ri^T
    Matrix3d RjT = Rj.transpose();          // Rj^T
    Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
    //Matrix3d JrInv_rPhi = SO3Calc::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
    Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
    Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

    // 1.
    // increment is the same as Forster 15'RSS
    // pi = pi + dpi,    pj = pj + dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
    // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

    // 2.
    // 15-Dim error vector order:
    // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
    // rPij - rVij - rPhiij - rBiasGi - rBiasAi
    //      Jacobian row order:
    // J_rPij_xxx
    // J_rVij_xxx
    // J_rPhiij_xxx
    // J_rBiasGi_xxx
    // J_rBiasAi_xxx

    // 3.
    // order in 'update_'
    // Vertex_i : dPi, dVi, dPhi_i, dBiasGyr_i, dBiasAcc_i,
    // Vertex_j : dPj, dVj, dPhi_j, dBiasGyr_j, dBiasAcc_j
    //      Jacobian column order:
    // J_xxx_dPi, J_xxx_dVi, J_xxx_dPhi_i, J_xxx_dBiasGyr_i, J_xxx_dBiasAcc_i
    // J_xxx_dPj, J_xxx_dVj, J_xxx_dPhi_j, J_xxx_dBiasGyr_j, J_xxx_dBiasAcc_j

    // 4.
    // For Vertex_i
    _jacobianOplusXi.setZero();

    // 4.1
    // J_rPij_xxx_i for Vertex_i
    _jacobianOplusXi.block<3,3>(0,0) = - RiT;//I3x3;      //J_rP_dpi
    _jacobianOplusXi.block<3,3>(0,3) = - RiT*dTij;  //J_rP_dvi
    //_jacobianOplusXi.block<3,3>(0,6) = SO3Calc::skew( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i
    _jacobianOplusXi.block<3,3>(0,6) = Sophus::SO3::hat( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i
    _jacobianOplusXi.block<3,3>(0,9) = - M.getJPBiasg();     //J_rP_dbgi
    _jacobianOplusXi.block<3,3>(0,12)= - M.getJPBiasa();     //J_rP_dbai

    // 4.2
    // J_rVij_xxx_i for Vertex_i
    _jacobianOplusXi.block<3,3>(3,0) = O3x3;    //dpi
    _jacobianOplusXi.block<3,3>(3,3) = - RiT;    //dvi
    //_jacobianOplusXi.block<3,3>(3,6) = SO3Calc::skew( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i
    _jacobianOplusXi.block<3,3>(3,6) = Sophus::SO3::hat( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i
    _jacobianOplusXi.block<3,3>(3,9) = - M.getJVBiasg();    //dbg_i
    _jacobianOplusXi.block<3,3>(3,12)= - M.getJVBiasa();    //dba_i

    // 4.3
    // J_rPhiij_xxx_i for Vertex_i
    //Matrix3d ExprPhiijTrans = SO3Calc::Expmap(rPhiij).transpose();  //Exp( rPhi )^T
    //Matrix3d JrBiasGCorr = SO3Calc::JacobianR(J_rPhi_dbg*dBgi);     //Jr( M.J_rPhi_bg * dBgi )
    Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
    Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg*dBgi);
    _jacobianOplusXi.block<3,3>(6,0) = O3x3;    //dpi
    _jacobianOplusXi.block<3,3>(6,3) = O3x3;    //dvi
    _jacobianOplusXi.block<3,3>(6,6) = - JrInv_rPhi * RjT * Ri;    //dphi_i
    _jacobianOplusXi.block<3,3>(6,9) = - JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
    _jacobianOplusXi.block<3,3>(6,12)= O3x3;    //dba_i

    // 4.4
    // J_rBiasGi_xxx_i for Vertex_i
    _jacobianOplusXi.block<3,3>(9,0) = O3x3;    //dpi
    _jacobianOplusXi.block<3,3>(9,3) = O3x3;    //dvi
    _jacobianOplusXi.block<3,3>(9,6) = O3x3;    //dphi_i
    _jacobianOplusXi.block<3,3>(9,9) = - I3x3;    //dbg_i
    _jacobianOplusXi.block<3,3>(9,12)= O3x3;    //dba_i

    // 4.5
    // J_rBiasAi_xxx_i for Vertex_i
    _jacobianOplusXi.block<3,3>(12,0) = O3x3;     //dpi
    _jacobianOplusXi.block<3,3>(12,3) = O3x3;     //dvi
    _jacobianOplusXi.block<3,3>(12,6) = O3x3;     //dphi_i
    _jacobianOplusXi.block<3,3>(12,9) = O3x3;    //dbg_i
    _jacobianOplusXi.block<3,3>(12,12)=  -I3x3;     //dba_i

    // 5.
    // For Vertex_j
    _jacobianOplusXj.setZero();

    // 5.1
    // J_rPij_xxx_j for Vertex_j
    _jacobianOplusXj.block<3,3>(0,0) = RiT;//*Rj;  //dpj
    _jacobianOplusXj.block<3,3>(0,3) = O3x3;    //dvj
    _jacobianOplusXj.block<3,3>(0,6) = O3x3;    //dphi_j
    _jacobianOplusXj.block<3,3>(0,9) = O3x3;    //dbg_j, all zero
    _jacobianOplusXj.block<3,3>(0,12)= O3x3;    //dba_j, all zero

    // 5.2
    // J_rVij_xxx_j for Vertex_j
    _jacobianOplusXj.block<3,3>(3,0) = O3x3;    //dpj
    _jacobianOplusXj.block<3,3>(3,3) = RiT;    //dvj
    _jacobianOplusXj.block<3,3>(3,6) = O3x3;    //dphi_j
    _jacobianOplusXj.block<3,3>(3,9) = O3x3;    //dbg_j, all zero
    _jacobianOplusXj.block<3,3>(3,12)= O3x3;    //dba_j, all zero

    // 5.3
    // J_rPhiij_xxx_j for Vertex_j
    _jacobianOplusXj.block<3,3>(6,0) = O3x3;    //dpj
    _jacobianOplusXj.block<3,3>(6,3) = O3x3;    //dvj
    _jacobianOplusXj.block<3,3>(6,6) = JrInv_rPhi;    //dphi_j
    _jacobianOplusXj.block<3,3>(6,9) = O3x3;    //dbg_j, all zero
    _jacobianOplusXj.block<3,3>(6,12)= O3x3;    //dba_j, all zero

    // 5.4
    // J_rBiasGi_xxx_j for Vertex_j
    _jacobianOplusXj.block<3,3>(9,0) = O3x3;     //dpj
    _jacobianOplusXj.block<3,3>(9,3) = O3x3;     //dvj
    _jacobianOplusXj.block<3,3>(9,6) = O3x3;     //dphi_j
    _jacobianOplusXj.block<3,3>(9,9) = I3x3;    //dbg_j
    _jacobianOplusXj.block<3,3>(9,12)= O3x3;     //dba_j

    // 5.5
    // J_rBiasAi_xxx_j for Vertex_j
    _jacobianOplusXj.block<3,3>(12,0) = O3x3;    //dpj
    _jacobianOplusXj.block<3,3>(12,3) = O3x3;    //dvj
    _jacobianOplusXj.block<3,3>(12,6) = O3x3;    //dphi_j
    _jacobianOplusXj.block<3,3>(12,9) = O3x3;    //dbg_j
    _jacobianOplusXj.block<3,3>(12,12)= I3x3;    //dba_j

    //std::cout<<"id "<<vi->id()<<" and "<<vj->id()<<" jacobians: "<<std::endl<<_jacobianOplusXi<<std::endl<<_jacobianOplusXj<<std::endl;
}

/**
 * @brief EdgeNavStatePointXYZ::EdgeNavStatePointXYZ
 */
EdgeNavStatePointXYZ::EdgeNavStatePointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>()
{
}
// Todo
bool EdgeNavStatePointXYZ::read(std::istream& is) {return true;}
bool EdgeNavStatePointXYZ::write(std::ostream& os) const {return true;}

void EdgeNavStatePointXYZ::linearizeOplus()
{
    const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const VertexNavState* vNavState = static_cast<const VertexNavState*>(_vertices[1]);

    const NavState& ns = vNavState->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of error w.r.t Pw
    _jacobianOplusXi = - Jpi * Rcb * Rwb.transpose();

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavState
    // order in 'update_': dP, dV, dPhi, dBiasGyr, dBiasAcc
    Matrix<double,2,15> JNavState = Matrix<double,2,15>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    //JNavState.block<2,3>(0,3) = 0;
    JNavState.block<2,3>(0,6) = JdRwb;
    //JNavState.block<2,3>(0,9) = 0;
    //JNavState.block<2,3>(0,12) = 0;

    // Jacobian of error w.r.t NavState
    _jacobianOplusXj = JNavState;
}

void EdgeNavStatePointXYZOnlyPose::linearizeOplus()
{
    const VertexNavState* vNavState = static_cast<const VertexNavState*>(_vertices[0]);

    const NavState& ns = vNavState->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    //const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];

    // Jacobian of camera projection
    Matrix<double,2,3> Maux;
    Maux.setZero();
    Maux(0,0) = fx;
    Maux(0,1) = 0;
    Maux(0,2) = -x/z*fx;
    Maux(1,0) = 0;
    Maux(1,1) = fy;
    Maux(1,2) = -y/z*fy;
    Matrix<double,2,3> Jpi = Maux/z;

    // error = obs - pi( Pc )
    // Pw <- Pw + dPw,          for Point3D
    // Rwb <- Rwb*exp(dtheta),  for NavState.R
    // Pwb <- Pwb + dPwb,   for NavState.P

    // Jacobian of Pc/error w.r.t dPwb
    //Matrix3d J_Pc_dPwb = -Rcb;
    Matrix<double,2,3> JdPwb = - Jpi * (-Rcb*Rwb.transpose());
    // Jacobian of Pc/error w.r.t dRwb
    Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
    //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
    Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

    // Jacobian of Pc w.r.t NavState
    // order in 'update_': dP, dV, dPhi, dBiasGyr, dBiasAcc
    Matrix<double,2,15> JNavState = Matrix<double,2,15>::Zero();
    JNavState.block<2,3>(0,0) = JdPwb;
    //JNavState.block<2,3>(0,3) = 0;
    JNavState.block<2,3>(0,6) = JdRwb;
    //JNavState.block<2,3>(0,9) = 0;
    //JNavState.block<2,3>(0,12) = 0;

    _jacobianOplusXi = JNavState;
}

/**
 * @brief VertexGyrBias::VertexGyrBias
 */
VertexGyrBias::VertexGyrBias() : BaseVertex<3, Vector3d>()
{
}

bool VertexGyrBias::read(std::istream& is)
{
    Vector3d est;
    for (int i=0; i<3; i++)
        is  >> est[i];
    setEstimate(est);
    return true;
}

bool VertexGyrBias::write(std::ostream& os) const
{
    Vector3d est(estimate());
    for (int i=0; i<3; i++)
        os << est[i] << " ";
    return os.good();
}

void VertexGyrBias::oplusImpl(const double* update_)  {
    Eigen::Map<const Vector3d> update(update_);
    _estimate += update;
    // Debug log
    //std::cout<<"updated bias estimate: "<<_estimate.transpose()<<", gyr bias update: "<<update.transpose()<<std::endl;
}

/**
 * @brief EdgeGyrBias::EdgeGyrBias
 */
EdgeGyrBias::EdgeGyrBias() : BaseUnaryEdge<3, Vector3d, VertexGyrBias>()
{
}


bool EdgeGyrBias::read(std::istream& is)
{
    return true;
}

bool EdgeGyrBias::write(std::ostream& os) const
{
    return true;
}

void EdgeGyrBias::computeError()
{
    const VertexGyrBias* v = static_cast<const VertexGyrBias*>(_vertices[0]);
    Vector3d bg = v->estimate();
    Matrix3d dRbg = Sophus::SO3::exp(J_dR_bg * bg).matrix();
    Sophus::SO3 errR ( ( dRbij * dRbg ).transpose() * Rwbi.transpose() * Rwbj ); // dRij^T * Riw * Rwj
    _error = errR.log();
    // Debug log
    //std::cout<<"dRbg: "<<std::endl<<dRbg<<std::endl;
    //std::cout<<"error: "<<_error.transpose()<<std::endl;
    //std::cout<<"chi2: "<<_error.dot(information()*_error)<<std::endl;
}


void EdgeGyrBias::linearizeOplus()
{
    Sophus::SO3 errR ( dRbij.transpose() * Rwbi.transpose() * Rwbj ); // dRij^T * Riw * Rwj
    Matrix3d Jlinv = Sophus::SO3::JacobianLInv(errR.log());

    _jacobianOplusXi = - Jlinv * J_dR_bg;

    // Debug log
    //std::cout<<"jacobian to bg:"<<std::endl<<_jacobianOplusXi<<std::endl;
    //std::cout<<"Jlinv: "<<Jlinv<<std::endl<<"J_dR_bg: "<<J_dR_bg<<std::endl;
}

}
