#include "single_point_simulator.h"

using namespace std;
using namespace COLIDE_RIGID;

void SinglePointSimulator::forward () {
  assert(quality_>1e-4);
  v_ += h_*extforce_/quality_;
  u_ = h_*v_;
}

void SinglePointSimulator::SetExtForce (Eigen::Vector3d &extforce) {
  extforce_ = extforce+gravity_;
}

void SinglePointSimulator::SetGravity (const Vector3d &g_normal)
{
  assert(quality_ > 1e-4);
  gravity_ = quality_ * g_normal;
}
