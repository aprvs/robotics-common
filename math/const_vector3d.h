#ifndef ROBOTICS_COMMON_MATH_VECTOR_H_
#define ROBOTICS_COMMON_MATH_VECTOR_H_

#include <cmath>
#include <cstdint>

#include "robotics-common/math/epsilons.h"
#include "robotics-common/math/geometry_defs.h"

namespace robotics_common {
namespace math {

/// @class ConstVector3d
/// @tparam Derived
template <typename Derived>
class ConstVector3d {
 public:
  double Get(Axis3d axis) const {
    switch (axis) {
      case Axis3d::kX:
        return GetX();
      case Axis3d::kY:
        return GetY();
      case Axis3d::kZ:
        return GetZ();
        // no default. Catch missing enums at compile time
    }
    return 0.0;
  }

  template <typename U>
  double Dot(const U& other) const {
    return GetX() * other.GetX() + GetY() * other.GetY() +
           GetZ() * other.GetZ();
  }

  double LengthSquared() const { return this->Dot(*this); }

  double Length() const { return std::sqrt(LengthSquared()); }

  bool IsZero(Epsilon epsilon) const {
    return Length() <= EpsilonValue(epsilon);
  }

  template <typename R, typename U>
  R Cross(const ConstVector3d<U>& other) const {
    R required_type;
    for (Axis3d axis : common::EnumListTrait<Axis3d>::values()) {
      Axis3d next_axis = NextCounterClockwiseAxis(axis);
      Axis3d next_next_axis = NextClockwiseAxis(axis);
      required_type.Set(axis,
                        this->Get(next_axis) * other.Get(next_next_axis) -
                            other.Get(next_axis) * this->Get(next_next_axis));
    }
    return required_type;
  }

  template <typename U>
  U Cross(const ConstVector3d<U>& other) const {
    return this->template Cross<U, U>(other);
  }

  template <typename U>
  bool GeometricallyEquals(const ConstVector3d<U>& other,
                           Epsilon epsilon) const {
    double difference = 0.0;
    for (Axis3d axis : common::EnumListTrait<Axis3d>::values()) {
      difference += std::pow(this->Get(axis) - other.Get(axis), 2);
    }
    return std::sqrt(difference) <= EpsilonValue(epsilon);
  }

  template <typename R>
  R operator*(double scalar) const {
    R result;
    for (Axis3d axis : common::EnumListTrait<Axis3d>::values()) {
      result.Set(axis, this->Get(axis) * scalar);
    }
    return result;
  }

  template <typename R, typename U>
  R operator+(const ConstVector3d<U>& other) const {
    R result;
    for (Axis3d axis : common::EnumListTrait<Axis3d>::values()) {
      result.Set(axis, this->Get(axis) + other.Get(axis));
    }
    return result;
  }

  template <typename U>
  U operator+(const ConstVector3d<U>& other) const {
    return this->template operator+<U, U>(other);
  }

  Derived operator-() const { return Derived(-GetX(), -GetY(), -GetZ()); }

  template <typename R, typename U>
  R operator-(const ConstVector3d<U>& other) const {
    R result = this->template operator+<R, U>(-other);
    return result;
  }

  template <typename U>
  U operator-(const ConstVector3d<U>& other) const {
    return this->template operator+<U, U>(-other);
  }

 protected:
  Derived const* ToDerived() const { return static_cast<Derived const*>(this); }

 private:
  inline double GetX() const { return ToDerived()->x(); }
  inline double GetY() const { return ToDerived()->y(); }
  inline double GetZ() const { return ToDerived()->z(); }
};

template <typename U>
inline typename std::enable_if<std::is_base_of<ConstVector3d<U>, U>::value,
                               U>::type
operator*(double scalar, const U& derived) {
  return derived * scalar;
}

template <typename T, typename U, typename V>
inline
    typename std::enable_if<std::is_base_of<ConstVector3d<T>, T>::value &&
                                std::is_base_of<ConstVector3d<U>, U>::value &&
                                std::is_base_of<ConstVector3d<V>, V>::value,
                            double>::type
    TripleProduct(const T& vector1, const U& vector2, const V& vector3) {
  return vector1.template Dot<U>(
      static_cast<ConstVector3d<U> const&>(vector2).template Cross<U, V>(
          vector3));
}

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_VECTOR_H_
