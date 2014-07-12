#include "collide_rigid_data.h"

#include <Eigen/Sparse>
#include <JsonFilePaser.h>
#include <MassMatrix.h>

using namespace COLIDE_RIGID;
using namespace Eigen;

void Plane::Collide (const UTILITY::VVec3d &nodes, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v) {
  for (int i=0; i<nodes.size(); ++i) {
    double depth = normal_.dot(point_ - nodes[i] - u.segment<3>(i*3));
    if (depth > 0) { // collision
      u.segment<3>(i*3) += depth * normal_;
      double change_v = (-kd-1)*normal_.dot(v.segment<3>(i*3));
      v.segment<3>(i*3) = v.segment<3>(i*3) - change_v*normal_;
    }
  }
}

void RigidBall::Collide (const UTILITY::VVec3d &nodes,  const double k, const Eigen::VectorXd &u, Eigen::VectorXd &extforce) const {
  cout << __FILE__ << __LINE__ << endl;
  exit(__LINE__);
}
int RigidBall::ExportObj (const string &filename) const {
    cout << __FILE__ << __LINE__ << endl;
  exit(__LINE__);

}
void RigidBall::transform (const Vector3d &dis) {
    cout << __FILE__ << __LINE__ << endl;
  exit(__LINE__);

}
Vector3d RigidBall::CalGravity (const double g) {
    cout << __FILE__ << __LINE__ << endl;
  exit(__LINE__);

}

int SenceData::InitDataFromFile (const char *ini_file) {
  LoadData(ini_file);
  CalTetMeshGravity();
}

int SenceData::LoadData (const char *ini_file) {
  ini_file_ = ini_file;
  UTILITY::JsonFilePaser jsonf;
  bool succ = jsonf.open(ini_file);
  assert(succ);

  succ &= jsonf.read("steps", steps_);
  succ &= jsonf.read("h", time_step_);
  succ &= jsonf.read("graverty_acceration", g_);
  {// output setting
    succ &= jsonf.read("output_steps", output_steps_);
    succ &= jsonf.read("output_ball_prefix", out_ball_prefix_);
    succ &= jsonf.read("output_tet_prefix", out_tet_mesh_prefix_);
    succ &= jsonf.read("output_shell_prefix", out_shell_mesh_prefix_);
  }
  string vol_file;
  string mtl_file;
  { // tet_mesh
    succ &= jsonf.read("tet_mesh", vol_file);
    succ &= jsonf.read("mtl",mtl_file);
  }
  assert(succ);
  tet_mesh_ = UTILITY::pTetMesh(new UTILITY::TetMesh());
  tet_mesh_->load(vol_file);
  tet_mesh_->loadElasticMtl(mtl_file);

  // @TODO load other data
  g_normal_ = Vector3d(0,0,1); // default gravity normal
  return 0;
}

int SenceData::CalTetMeshGravity () {
  UTILITY::MassMatrix mass;
  DiagonalMatrix<double, -1> M;
  mass.compute(M, *tet_mesh_);
  assert(M.size() == tet_mesh_->nodes().size()*3);
  tet_mesh_gravity_.resize(M.size());
  tet_mesh_gravity_.setOnes();
  tet_mesh_gravity_= g_*tet_mesh_gravity_*M;
  return 0;
}
