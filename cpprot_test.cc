#include "cpprot.h"

/**
 * @function main
 * @brief main function
 */
int main(int argc, char **argv)
{
    Vector3d vec = Vector3d::UnitX();

    double angle1 = M_PI/3.0;
    
    Vector3d axis1;
    axis1 << 0.0, 0.0, 1.0;
    Quaterniond quat1 = CppRot::AngleAxis2Quat(angle1, axis1);
    Matrix3d Tmat1 = CppRot::Quat2Tmat(quat1);
    Matrix3d Rmat1 = CppRot::Quat2Rmat(quat1);

    double angle2 = M_PI/4.0;

    Vector3d axis2 = Tmat1*Vector3d::UnitX();
    Quaterniond quat2 = CppRot::AngleAxis2Quat(angle2, axis2);
    Matrix3d Tmat2 = CppRot::Quat2Tmat(quat2);
    Matrix3d Rmat2 = CppRot::Quat2Rmat(quat2);

    Vector3d vec_passive_Tmat = Tmat2*Tmat1*vec;
    Vector3d vec_passive_quat = CppRot::Quat2Tmat(CppRot::QuatMult_S(quat2, quat1))*vec;

    Vector3d vec_active_Rmat = Rmat2*Rmat1*vec;
    Vector3d vec_active_quat = CppRot::Quat2Rmat(CppRot::QuatMult_H(quat2, quat1))*vec;

    std::cout << vec_passive_Tmat.transpose() << std::endl;
    std::cout << vec_passive_quat.transpose() << std::endl;

    std::cout << std::endl;

    std::cout << vec_active_Rmat.transpose() << std::endl;
    std::cout << vec_active_quat.transpose() << std::endl;

    std::cout << std::endl;

    std::cout << CppRot::QuatMult_S(quat2, quat1).w() << "\t" << CppRot::QuatMult_S(quat2, quat1).vec().transpose() << std::endl;
    std::cout << CppRot::QuatMult_H(quat1, quat2).w() << "\t" << CppRot::QuatMult_H(quat1, quat2).vec().transpose() << std::endl;

    std::cout << std::endl;

    Matrix3d Tmat = CppRot::Quat2Tmat(quat1.normalized());
    Matrix3d Rmat = CppRot::Quat2Rmat(quat1.normalized());

    std::cout << quat1.w() << " " << quat1.vec().transpose() << std::endl;
    std::cout << std::endl;
    std::cout << Tmat << std::endl;
    std::cout << std::endl;
    std::cout << Rmat << std::endl;

    return 0;
};
