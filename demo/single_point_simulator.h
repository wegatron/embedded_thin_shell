#ifndef _SINGLE_POINT_SIMULATOR_H_
#define _SINGLE_POINT_SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace COLIDE_RIGID {
  class SinglePointSimulator
  {
  public:
    void forward ();

    // external force, do not need add gravity
    void SetExtForce (const Eigen::Vector3d &extforce);
    void CalSetGravity (const Eigen::Vector3d &g_normal);
    void SetTimeStep (double h) { h_=h; }

    void SetQuality (double quality) { quality_=quality; }
    void SetV (const Eigen::Vector3d& v) { v_=v; }

    const Eigen::Vector3d &GetU () const { return u_; }
    Eigen::Vector3d &GetV () { return v_; }
  private:
    double h_; // time step
    double quality_;
    Eigen::Vector3d u_; // displacement in single time step not accumlate
    Eigen::Vector3d v_;
    Eigen::Vector3d extforce_;
    Eigen::Vector3d gravity_;
    Eigen::Vector3d join_force_;
  };
  typedef boost::shared_ptr<SinglePointSimulator> pSinglePointSimulator;
}

#endif /* _SINGLE_POINT_SIMULATOR_H_ */
