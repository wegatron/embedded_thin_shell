#include "single_point_simulator.h"
#include "zsw_debug.h"
using namespace std;
using namespace COLIDE_RIGID;

void SinglePointSimulator::forward () {
  assert(quality_>1e-4);
  v_ += h_*join_force_/quality_;
  u_ = h_*v_;
}

void SinglePointSimulator::SetExtForce (const Eigen::Vector3d &extforce) {
  extforce_ = extforce;
  join_force_ = extforce_ + gravity_;
  //ZSW_REDUNDANT("join force:" << join_force_.transpose());
}

void SinglePointSimulator::CalSetGravity (const Eigen::Vector3d &g_normal)
{
  assert(quality_ > 1e-4);
  gravity_ = quality_ * g_normal;
  join_force_ = extforce_ + gravity_;
  ZSW_REDUNDANT("join force:" << join_force_.transpose());
}
