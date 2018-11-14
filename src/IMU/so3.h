// This file is part of Sophus.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef SOPHUS_SO3_H
#define SOPHUS_SO3_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>


namespace Sophus
{
using namespace Eigen;

const double SMALL_EPS = 1e-10;

class SO3
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Jr, right jacobian of SO(3)
  static Matrix3d JacobianR(const Vector3d& w);
  // Jr^(-1)
  static Matrix3d JacobianRInv(const Vector3d& w);
  // Jl, left jacobian of SO(3), Jl(x) = Jr(-x)
  static Matrix3d JacobianL(const Vector3d& w);
  // Jl^(-1)
  static Matrix3d JacobianLInv(const Vector3d& w);

  // ----------------------------------------

  SO3                        ();

  SO3                        (const SO3 & other);

  explicit
  SO3                        (const Matrix3d & _R);

  explicit
  SO3                        (const Quaterniond & unit_quaternion);

  SO3                        (double rot_x,
                              double rot_y,
                              double rot_z);
  void
  operator=                  (const SO3 & so3);

  SO3
  operator*                  (const SO3 & so3) const;

  void
  operator*=                 (const SO3 & so3);

  Vector3d
  operator*                  (const Vector3d & xyz) const;

  SO3
  inverse                    () const;

  Matrix3d
  matrix                     () const;

  Matrix3d
  Adj                        () const;

  Matrix3d
  generator                  (int i);

  Vector3d
  log                        () const;

  static SO3
  exp                        (const Vector3d & omega);

  static SO3
  expAndTheta                (const Vector3d & omega,
                              double * theta);
  static Vector3d
  log                        (const SO3 & so3);

  static Vector3d
  logAndTheta                (const SO3 & so3,
                              double * theta);

  static Matrix3d
  hat                        (const Vector3d & omega);

  static Vector3d
  vee                        (const Matrix3d & Omega);

  static Vector3d
  lieBracket                 (const Vector3d & omega1,
                              const Vector3d & omega2);

  static Matrix3d
  d_lieBracketab_by_d_a      (const Vector3d & b);

  void
  setQuaternion              (const Quaterniond& quaternion);

  const Quaterniond & unit_quaternion() const
  {
    return unit_quaternion_;
  }

  static const int DoF = 3;

protected:
  Quaterniond unit_quaternion_;
};

inline std::ostream& operator <<(std::ostream & out_str,
                                 const SO3 & so3)
{

  out_str << so3.log().transpose() << std::endl;
  return out_str;
}

} // end namespace


#endif
