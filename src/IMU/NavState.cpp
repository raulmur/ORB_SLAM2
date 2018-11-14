#include "NavState.h"

namespace ORB_SLAM2
{

NavState::NavState()
{
    //_qR.setIdentity();     // rotation
    _P.setZero();         // position
    _V.setZero();         // velocity

    _BiasGyr.setZero();   // bias of gyroscope
    _BiasAcc.setZero();   // bias of accelerometer

    _dBias_g.setZero();
    _dBias_a.setZero();
}

// if there's some other constructor, normalizeRotation() is needed
NavState::NavState(const NavState &_ns):
    _P(_ns._P), _V(_ns._V), _R(_ns._R),
    _BiasGyr(_ns._BiasGyr), _BiasAcc(_ns._BiasAcc),
    _dBias_g(_ns._dBias_g), _dBias_a(_ns._dBias_a)
{
    //normalizeRotation();
}

void NavState::IncSmall(Vector15d update)
{
    // 1.
    // order in 'update_'
    // dP, dV, dPhi, dBiasGyr, dBiasAcc

    // 2.
    // the same as Forster 15'RSS
    // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
    // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

    Vector3d upd_P = update.segment<3>(0);
    Vector3d upd_V = update.segment<3>(3);
    Vector3d upd_Phi = update.segment<3>(6);
    Vector3d upd_dBg = update.segment<3>(9);
    Vector3d upd_dBa = update.segment<3>(12);

    // rotation matrix before update
    //Matrix3d R = Get_qR().toRotationMatrix();
    //Matrix3d R = Get_R().matrix();

    // position
    _P += upd_P;
    // velocity
    _V += upd_V;
    // rotation
    //Matrix3d dR = Sophus::SO3::exp(upd_Phi).matrix();
    Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
    _R = Get_R()*dR;
    //_qR = Quaterniond(R*dR);
    //normalizeRotation();    // remember to normalize rotation
    // delta bias of gyroscope
    _dBias_g += upd_dBg;
    // delta bias of accelerometer
    _dBias_a += upd_dBa;

}


void NavState::IncSmallPR(Vector6d dPR)
{
    Vector3d upd_P = dPR.segment<3>(0);
    Vector3d upd_Phi = dPR.segment<3>(3);

    _P += upd_P;
    _R = _R * Sophus::SO3::exp(upd_Phi);
}

void NavState::IncSmallV(Vector3d dV)
{
    _V += dV;
}

void NavState::IncSmallPVR(Vector9d updatePVR)
{
    // Update P/V/R in NavState

    // 1.
    // order in 'update_'
    // dP, dV, dPhi

    // 2.
    // the same as Forster 15'RSS
    // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)

    Vector3d upd_P = updatePVR.segment<3>(0);
    Vector3d upd_V = updatePVR.segment<3>(3);
    Vector3d upd_Phi = updatePVR.segment<3>(6);

    // rotation matrix before update
    Matrix3d R = Get_R().matrix();

    // position
    _P += upd_P;
    // velocity
    _V += upd_V;
    // rotation
    Sophus::SO3 dR = Sophus::SO3::exp(upd_Phi);
    _R = Get_R()*dR;

}

void NavState::IncSmallBias(Vector6d updatedBias)
{
    // Update bias in NavState

    // 1.
    // order in 'update_'
    // dBiasGyr, dBiasAcc

    // 2.
    //      Note: the optimized bias term is the 'delta bias'
    // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
    // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

    Vector3d upd_dBg = updatedBias.segment<3>(0);
    Vector3d upd_dBa = updatedBias.segment<3>(3);

    // delta bias of gyroscope
    _dBias_g += upd_dBg;
    // delta bias of accelerometer
    _dBias_a += upd_dBa;

}

}
