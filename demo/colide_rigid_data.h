#ifndef _COLIDE_RIGID_DATA_H_
#define _COLIDE_RIGID_DATA_H_

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <Objmesh.h>
#include <TetMesh.h>

namespace COLIDE_RIGID {

  class Plane {
  public:
    void Collide (const UTILITY::VVec3d &nodes, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v);
  private:
    Eigen::Vector3d point_;
    Eigen::Vector3d normal_;
  };
  typedef boost::shared_ptr<Plane> pPlane;

  class RigidBall {
  public:
    void Collide (const UTILITY::VVec3d &nodes,  const double k, const Eigen::VectorXd &u, Eigen::VectorXd &extforce);
    int ExportObj (const string &filename) const;
    void transform (const Vector3d &dis);
    Vector3d CalGravity (const double g);
  private:
    double r_;
    double density_;
    Eigen::Vector3d center_;
    Eigen::VectorXd u_;
    UTILITY::Objmesh obj_;
  };
  typedef boost::shared_ptr<RigidBall> pRigidBall;

  class SenceData {
  public:
    int InitDataFromFile (const char *ini_file);
    int LoadData (const char *ini_file);
    int CalTetMeshGravity ();

    int steps_;
    int output_steps_;
    double time_step_;
    double kd_; // kd for velocity
    double k_; // stiffness k for collision force
    double g_; // gravity value
    Eigen::Vector3d g_normal_; // gravity normal
    string out_ball_prefix_;
    string out_tet_mesh_prefix_;
    string out_shell_mesh_prefix_;

    Eigen::VectorXd tet_mesh_gravity_;
    UTILITY::pTetMesh tet_mesh_;

    RigidBall rigid_ball_;
    std::vector<pPlane> plans_;
  };
  typedef boost::shared_ptr<SenceData> pSenceData;
}

#endif /* _COLIDE_RIGID_DATA_H_ */
