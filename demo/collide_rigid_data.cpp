#include "collide_rigid_data.h"

#include <Eigen/Sparse>
#include <JsonFilePaser.h>
#include <assertext.h>
#include <MassMatrix.h>

#include "zsw_debug.h"

using namespace COLIDE_RIGID;
using namespace Eigen;

void Plane::Collide (const UTILITY::VVec3d &nodes, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v) {
  for (int i=0; i<nodes.size(); ++i) {
    double depth = normal_.dot(point_ - nodes[i] - u.segment<3>(i*3));
    if (depth > 0) { // collision
      u.segment<3>(i*3) += depth * normal_;
      double change_v = (-kd-1)*normal_.dot(v.segment<3>(i*3));
      assert(change_v > 0);
      v.segment<3>(i*3) = v.segment<3>(i*3) + change_v*normal_;
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
  int ret = LoadData(ini_file);
  if (ret == 0) {
    ret = CalTetMeshGravity();
  }
  return ret;
}

int SenceData::LoadData (const char *ini_file) {
  ini_file_ = ini_file;
  UTILITY::JsonFilePaser jsonf;
  bool succ = jsonf.open(ini_file);
  assert(succ);

  {
    succ &= jsonf.read("steps", steps_);
    succ &= jsonf.read("h", time_step_);
    succ &= jsonf.read("graverty_acceration", g_);

    // output setting
    succ &= jsonf.read("output_steps", output_steps_);
    succ &= jsonf.read("output_ball_prefix", out_ball_prefix_);
    succ &= jsonf.read("output_tet_prefix", out_tet_mesh_prefix_);
    succ &= jsonf.read("output_shell_prefix", out_shell_mesh_prefix_);
  }
  succ &= jsonf.read("kd", kd_);
  assert(succ);
  g_normal_ = Vector3d(0,0,-1); // default gravity normal


  { // tet_mesh
    string vol_file;
    string mtl_file;
    succ &= jsonf.readFilePath("vol_file", vol_file);
    succ &= jsonf.readFilePath("elastic_mtl",mtl_file);
    assert(succ);
    tet_mesh_ = UTILITY::pTetMesh(new UTILITY::TetMesh());
    tet_mesh_->load(vol_file);
    tet_mesh_->loadElasticMtl(mtl_file);
  }

  int ret=0;
  {// load planes
    string plane_file;
    jsonf.readFilePath("plane_file", plane_file);
    ret = LoadPlanes(plane_file);
    assert(ret == 0);
  }
  // @TODO load rigid ball

  if (succ) { return ret; }
  else {
    ZSW_DEBUG("load setting succ not true");
    return __LINE__;
  }
}
int SenceData::LoadPlanes (const string &file) {
  ifstream ifs(file);
  if (!ifs) {
    cerr << "cannot open file:" << file << endl;
    return __LINE__;
  }
  int plane_num;
  ifs >> plane_num;
  for (int i=0; i<plane_num; ++i) {
    pPlane plane_ptr = pPlane(new Plane());
    Vector3d &tmp_point = plane_ptr->GetPoint();
    Vector3d &tmp_normal = plane_ptr->GetNormal();
    ifs >> tmp_point[0] >> tmp_point[1] >> tmp_point[2];
    ifs >> tmp_normal[0] >> tmp_normal[1] >> tmp_normal[2];
    planes_.push_back(plane_ptr);
  }
  return 0;
}

int SenceData::CalTetMeshGravity () {
  UTILITY::MassMatrix mass;
  DiagonalMatrix<double, -1> M;
  mass.compute(M, *tet_mesh_);

  assert_eq(M.diagonal().size(), tet_mesh_->nodes().size()*3);

  tet_mesh_gravity_.resize(3, tet_mesh_->nodes().size());
  tet_mesh_gravity_.row(0).setConstant(g_normal_[0]);
  tet_mesh_gravity_.row(1).setConstant(g_normal_[1]);
  tet_mesh_gravity_.row(2).setConstant(g_normal_[2]);

  tet_mesh_gravity_.resize(M.diagonal().size(), 1);

  assert_eq_eps(tet_mesh_gravity_(0,0), g_normal_[0], 1e-3);
  assert_eq_eps(tet_mesh_gravity_(1,0), g_normal_[1], 1e-3);
  assert_eq_eps(tet_mesh_gravity_(2,0), g_normal_[2], 1e-3);

  tet_mesh_gravity_= g_*M*tet_mesh_gravity_;

  return 0;
}
