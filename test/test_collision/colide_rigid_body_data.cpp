#include "colide_rigid_body_data.h"

using namespace colide_rigid_body;

void Plane::Collide (const VVec3d &nodes, const Plane &plane, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v) {
  for (int i=0; i<nodes.size(); ++i) {
    double depth = normal.dot(point_ - nodes[i] - u.segment<3>(i*3));
    if (depth > 0) { // collision
      u.segment<3>(i*3) += depth * normal_;
      double change_v = (-kd-1)*normal_.dot(v.segment<3>(i*3));
      v.segment<3>(i*3) = v.segment<3>(i*3) - change_v*normal;
    }
  }
}
