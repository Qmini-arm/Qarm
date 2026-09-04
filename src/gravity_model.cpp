#include "qmini_arm/gravity_model.hpp"

#include <cmath>
#include <stdexcept>

namespace qmini_arm {
namespace {

struct Vec3 {
  double x;
  double y;
  double z;
};

struct Mat3 {
  double value[3][3];
};

struct Transform {
  Mat3 rotation;
  Vec3 position;
};

Vec3 add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(const Vec3& value, double factor) {
  return {factor * value.x, factor * value.y, factor * value.z};
}

double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
  Mat3 result{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int inner = 0; inner < 3; ++inner) {
        result.value[row][column] +=
            a.value[row][inner] * b.value[inner][column];
      }
    }
  }
  return result;
}

Vec3 multiply(const Mat3& matrix, const Vec3& vector) {
  return {
      matrix.value[0][0] * vector.x + matrix.value[0][1] * vector.y +
          matrix.value[0][2] * vector.z,
      matrix.value[1][0] * vector.x + matrix.value[1][1] * vector.y +
          matrix.value[1][2] * vector.z,
      matrix.value[2][0] * vector.x + matrix.value[2][1] * vector.y +
          matrix.value[2][2] * vector.z,
  };
}

Mat3 transpose(const Mat3& matrix) {
  Mat3 result{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.value[row][column] = matrix.value[column][row];
    }
  }
  return result;
}

Mat3 rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
           {-sp, cp * sr, cp * cr}}};
}

Mat3 rotationX(double angle) {
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return {{{1.0, 0.0, 0.0},
           {0.0, cosine, -sine},
           {0.0, sine, cosine}}};
}

Transform compose(const Transform& parent, const Transform& child) {
  return {
      multiply(parent.rotation, child.rotation),
      add(parent.position, multiply(parent.rotation, child.position)),
  };
}

Vec3 transformPoint(const Transform& transform, const Vec3& point) {
  return add(transform.position, multiply(transform.rotation, point));
}

const Transform kJointOrigins[6] = {
    {rpy(0.0, 0.0, 0.0), {0.0, 0.0, 0.0}},
    {rpy(0.7259, 0.844114116, -1.570796327),
     {0.082449884, 0.000464880, 0.000523357}},
    {rpy(1.453370820, 0.0, -3.141592654),
     {0.013000008, 0.224214994, -0.199318371}},
    {rpy(1.45178, 0.0, -3.141592654),
     {0.013000008, -0.224214968, 0.199318367}},
    {rpy(-0.844107722, 0.844114116, -1.570796327),
     {0.082449884, 0.000464880, 0.000523357}},
    {rpy(0.7259, -0.844114116, 1.570796327),
     {0.083950124, -0.000465277, -0.000522985}},
};

const Vec3 kLinkCentersOfMass[6] = {
    {0.076902896, -0.000540190, -0.000692581},
    {0.016355333, 0.193045983, -0.171512178},
    {0.016353625, -0.193059219, 0.171524730},
    {0.076904739, -0.000544010, -0.000695119},
    {0.078231092, 0.000626505, 0.000621803},
    {0.021295905, -0.013071630, 0.011629218},
};

const double kLinkMassesKg[6] = {
    0.567450382,
    0.676212997,
    0.676213000,
    0.567450400,
    0.567450400,
    0.016653600,
};

}  // namespace

JointVector GravityModel::compensationTorque(
    const JointVector& joint_position_rad) const {
  for (double value : joint_position_rad) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("joint position contains a non-finite value");
    }
  }

  // URDF world_to_base rotates the base relative to world. Express world -Z
  // gravity in base_link coordinates before traversing the serial chain.
  const Mat3 world_from_base = rpy(0.74, -1.57, 0.0);
  const Vec3 gravity_base =
      multiply(transpose(world_from_base), Vec3{0.0, 0.0, -9.80665});

  const Mat3 identity = {{{1.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0},
                          {0.0, 0.0, 1.0}}};
  Transform parent{identity, {0.0, 0.0, 0.0}};
  Vec3 joint_origins[6]{};
  Vec3 joint_axes[6]{};
  Vec3 centers_of_mass[6]{};

  for (int index = 0; index < 6; ++index) {
    const Transform joint_frame = compose(parent, kJointOrigins[index]);
    joint_origins[index] = joint_frame.position;
    joint_axes[index] =
        multiply(joint_frame.rotation, Vec3{1.0, 0.0, 0.0});
    const Transform child{
        multiply(joint_frame.rotation, rotationX(joint_position_rad[index])),
        joint_frame.position,
    };
    centers_of_mass[index] =
        transformPoint(child, kLinkCentersOfMass[index]);
    parent = child;
  }

  JointVector result{};
  for (int joint = 0; joint < 6; ++joint) {
    double gravity_generalized_force = 0.0;
    for (int link = joint; link < 6; ++link) {
      const Vec3 lever = subtract(centers_of_mass[link], joint_origins[joint]);
      const Vec3 force = scale(gravity_base, kLinkMassesKg[link]);
      gravity_generalized_force +=
          dot(joint_axes[joint], cross(lever, force));
    }
    result[joint] = -gravity_generalized_force;
  }
  return result;
}

}  // namespace qmini_arm
