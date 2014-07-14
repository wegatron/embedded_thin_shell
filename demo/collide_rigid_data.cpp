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
  if(obj_.write(filename)) { return 0; }
  ZSW_INFO("Cannot export file:" << filename);
  return __LINE__;
}

bool RigidBall::LoadObj (const string &filename) {
  return obj_.load(filename);
}
void RigidBall::transform (const Vector3d &dis) {
  VectorXd &verts = obj_.getModifyVerts();
  assert(verts.size()%3 == 0);
  MatrixXd tmp_disp(3,verts.size()/3);
  tmp_disp.row(0).setConstant(dis[0]);
  tmp_disp.row(1).setConstant(dis[1]);
  tmp_disp.row(2).setConstant(dis[2]);
  tmp_disp.resize(verts.size(), 1);
  assert_eq_eps(tmp_disp(0,0), dis[0], 1e-4);
  assert_eq_eps(tmp_disp(1,0), dis[1], 1e-4);
  assert_eq_eps(tmp_disp(2,0), dis[2], 1e-4);
  ZSW_REDUNDANT(tmp_disp.transpose());
  verts += tmp_disp;
}
Vector3d RigidBall::CalGravity (const Vector3d &g_normal) {
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
    vector<double> g_v;
    succ &= jsonf.read("graverty", g_v);
    assert(g_v.size()==3);
    std::copy(g_v.begin(), g_v.end(), &g_normal_[0]);
    ZSW_INFO("gravity" << g_normal_.transpose());
    // output setting
    succ &= jsonf.read("output_steps", output_steps_);
    succ &= jsonf.read("output_ball_prefix", out_ball_prefix_);
    succ &= jsonf.read("output_tet_prefix", out_tet_mesh_prefix_);
    succ &= jsonf.read("output_shell_prefix", out_shell_mesh_prefix_);
  }
  succ &= jsonf.read("kd", kd_);
  assert(succ);

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

  {// load planes
    vector<double> plane_v;
    succ &= jsonf.read("planes", plane_v);
    assert(plane_v.size()%6 == 0);
    for (vector<double>::iterator it=plane_v.begin(); it!=plane_v.end(); it+=6) {
      pPlane plane_ptr = pPlane(new Plane());
      std::copy(it, it+3, &plane_ptr->GetPoint()[0]);
      std::copy(it+3, it+6, &plane_ptr->GetNormal()[0]);
      cout << *plane_ptr << endl;
      planes_.push_back(plane_ptr);
    }
  }
  // @TODO load rigid ball
  {
    string ball_file;
    succ &= jsonf.readFilePath("ball_file", ball_file);
    succ &= rigid_ball_.LoadObj(ball_file);
    assert(succ);
  }
  assert(succ);
  ZSW_DEBUG(!succ, "load setting error: succ not true");
  return succ ? 0:__LINE__;
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

  tet_mesh_gravity_= M*tet_mesh_gravity_;

  return 0;
}

std::ostream& COLIDE_RIGID::operator<<(std::ostream& os, Plane &plane) {
  os << "[plane] point: " << plane.point_.transpose() << " normal:" << plane.normal_.transpose();
   return os;
}
