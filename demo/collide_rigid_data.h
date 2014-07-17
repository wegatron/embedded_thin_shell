#ifndef _COLLIDE_RIGID_DATA_H_
#define _COLLIDE_RIGID_DATA_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <Objmesh.h>
#include <TetMesh.h>

namespace COLIDE_RIGID {

  class RigidBall {
  public:
    bool InitFromObj (const string &filename);

    void Collide (const UTILITY::VVec3d &nodes,  const double k, const Eigen::VectorXd &u, Eigen::VectorXd &extforce) const;
    void Transform (const Vector3d &dis);

    double GetR () const { return r_; }
    const Eigen::Vector3d &GetCenter () const { return center_; }
    double GetQuality () const { return quality_; }
    void SetDensity (const double density) { density_=density; }

    int ExportObj (const string &filename) const;
    int ExportVtk (const string &filename) const;
  private:
    void CalSetCRQ (); // caculate and set center and radius and quality
    double r_;
    double quality_;

    double density_;
    Eigen::Vector3d center_;
    UTILITY::Objmesh obj_;
  };
  typedef boost::shared_ptr<RigidBall> pRigidBall;

  class Plane {
  public:
    void Collide (const UTILITY::VVec3d &nodes, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v);
    void Collide (const double kd, Eigen::Vector3d &v, RigidBall &rigid_ball);
    friend ostream& operator<<(ostream&, Plane&);
    Eigen::Vector3d &GetPoint () { return point_; }
    Eigen::Vector3d &GetNormal () { return normal_; }
  private:
    Eigen::Vector3d point_;
    Eigen::Vector3d normal_;
  };
  typedef boost::shared_ptr<Plane> pPlane;

  class SenceData {
  public:
    int InitDataFromFile (const char *ini_file);

    int steps_;
    int output_steps_;

    int subdivision_time_;

    double time_step_;
    double soft_kd_; // kd for soft
    double rigid_kd_; // kd for rigid
    double stiff_k_; // stiffness k for collision force
    Eigen::Vector3d g_normal_; // gravity normal
    string ini_file_; // init file path
    string out_ball_prefix_;
    string out_tet_mesh_prefix_;
    string out_shell_mesh_prefix_;

    Eigen::MatrixXd tet_mesh_gravity_;
    UTILITY::pTetMesh tet_mesh_;

    RigidBall rigid_ball_;
    Eigen::Vector3d ball_v_;
    std::vector<pPlane> planes_;
  private:
    int LoadData (const char *ini_file);
    int CalTetMeshGravity ();
  };
  typedef boost::shared_ptr<SenceData> pSenceData;

  ostream& operator<<(ostream&, Plane&);
}

#endif /* _COLIDE_RIGID_DATA_H_ */
