#ifndef _COLLISIONFORCETETFULL_H_
#define _COLLISIONFORCETETFULL_H_

#include <eigen3/Eigen/Sparse>

namespace UTILITY{
  const double check_inball_eps = 1e-6;
  /**
   * @class CollisionForceTetFull base class for full elastic forces of
   * tetrahedron mesh.
   */
  class CollisionForceTetFull{
  public:
    CollisionForceTetFull(){}
    virtual double energy(const Eigen::VectorXd &X) const =0;
    virtual void force(const Eigen::VectorXd &X, Eigen::VectorXd &out) const =0;
    virtual const Eigen::SparseMatrix<double> &K(const Eigen::VectorXd &X)=0;
    virtual void judge(const Eigen::VectorXd &X, Eigen::VectorXd &vertex_weight) const =0;
  };

  class CollisionBallForceTetFull : public CollisionForceTetFull {
  public:
  CollisionBallForceTetFull(const double k) :_k(k) {}
    // inheritance
    double energy(const Eigen::VectorXd &X) const;
    void force(const Eigen::VectorXd &X, Eigen::VectorXd &out) const;
    const Eigen::SparseMatrix<double> &K(const Eigen::VectorXd &X);
    void judge(const Eigen::VectorXd &X, Eigen::VectorXd &vertex_weight) const;

    // self
    void setBallCenter(const Eigen::Vector3d &ball_center) { _ball_center = ball_center; }
    void setR(const double r) { _r = r; }
    void setCoeffK(const double k) { _k = k; }
  private:
    // self
    inline int inball(const Eigen::Vector3d &vertex) const {
      return (vertex-_ball_center).squaredNorm()<_r*_r;
    }
    Eigen::Vector3d _ball_center;
    double _r;
    double _k;
    Eigen::SparseMatrix<double> _K;
  };
}

#endif
