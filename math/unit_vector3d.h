#ifndef ROBOTICS_COMMON_MATH_UNIT_VECTOR3D_H_
#define ROBOTICS_COMMON_MATH_UNIT_VECTOR3D_H_

#include "common/error_or.h"
#include "robotics-common/math/const_vector3d.h"
#include "robotics-common/math/epsilons.h"

namespace robotics_common {
namespace math {

class UnitVector3d : public ConstVector3d<UnitVector3d> {
 public:
  UnitVector3d();

  static const UnitVector3d kXAxis;
  static const UnitVector3d kYAxis;
  static const UnitVector3d kZAxis;
  static const UnitVector3d kAxes[3];

  static common::ErrorOr<UnitVector3d> NormalizeAndCreate(double x, double y,
                                                          double z,
                                                          Epsilon epsilon);

  UnitVector3d(const UnitVector3d&) = default;
  UnitVector3d& operator=(const UnitVector3d&) = default;

  UnitVector3d Cross(const UnitVector3d& other) const;

  template <typename U>
  U Cross(const U& other) const {
    return static_cast<ConstVector3d<UnitVector3d> const*>(this)->Cross<U, U>(
        other);
  }

  void Negate();

  double Angle(const UnitVector3d& other) const;

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

 private:
  friend class ConstVector3d<UnitVector3d>;
  UnitVector3d(double x, double y, double z);
  void Set(Axis3d axis, double value);

  double x_;
  double y_;
  double z_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_UNIT_VECTOR3D_H_
