#ifndef _SINGLE_POINT_SIMULATOR_H_
#define _SINGLE_POINT_SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace COLIDE_RIGID {
  class SinglePointSimulator
  {
  public:
    void forward ();
    void SetExtForce (Eigen::Vector3d &extforce);
    void SetTimeStep (double h) { h_=h; }
    void SetQuality (double quality) { quality_=quality; }
    const Eigen::Vector3d &getU () const { return u_; }
  private:
    double h_; // time step
    double quality_;
    Eigen::Vector3d u_; // displacement in single time step not accumlate
    Eigen::Vector3d v_;
    Eigen::Vector3d extforce_;
  };
  typedef boost::shared_ptr<SinglePointSimulator> pSinglePointSimulator;
}

#endif /* _SINGLE_POINT_SIMULATOR_H_ */
