#ifndef _COLIDE_RIGID_BODY_DATA_H_
#define _COLIDE_RIGID_BODY_DATA_H_

#include <vector>
#include <Eigen/Dense>

namespace colide_rigid_body {

  class Plane {
  public:
    void Collide (const VVec3d &nodes, const Plane &plane, const double kd, Eigen::VectorXd &u, Eigen::VectorXd &v);
  private:
    Eigen::Vector3d point_;
    Eigen::Vector3d normal_;
  };

  class RigdBody {
  public:
    void Collide (const VVec3d &nodes,  const double k, Eigen::VectorXd &u, Eigen::VectorXd &extforce);
    int ExoprtObj (string filename);
  private:
    Eigen::VectorXd v;
    Eigen::VectorXd x;
  };

  class SimData {
  public:
    int InitDataFromFile (const char *ini_file);
  private:
    int LoadData (const char *ini_file);
    int CalTetMeshGravity ();
    int steps_;
    int output_steps_;
    double kd;

    int gravity_axis_index_; // 0: x axis; 1: y axis; 2: z axis.
    bool isgravity_negtive_; // true: gavity is negative; false: gavity is positive.

    Eigen::VectorXd tet_mesh_gravity;
    pTetMesh tet_mesh_;

    RigdBody rigid_body_;
    std::vector<Plane> plans_;
  };
};

#endif /* _COLIDE_RIGID_BODY_DATA_H_ */
