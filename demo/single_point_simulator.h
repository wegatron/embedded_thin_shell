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
  private:
    double quality_;
    Eigen::Vector3d u_;
    Eigen::Vector3d v_;
    Eigen::Vector3d extforce_;
  };
  typedef boost::shared_ptr<SinglePointSimulator> pSinglePointSimulator;
}

#endif /* _SINGLE_POINT_SIMULATOR_H_ */
