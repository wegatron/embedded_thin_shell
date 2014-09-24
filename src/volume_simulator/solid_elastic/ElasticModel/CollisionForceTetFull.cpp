#include "CollisionForceTetFull.h"

#include <iostream>
#include "collision/collision_ball_energy.h"

#define EPS 1e-6

void UTILITY::CollisionBallForceTetFull::judge(const Eigen::VectorXd &X,
                                           Eigen::VectorXd &vertex_weight) const
{
  assert(X.size()%3 == 0);
  int n_vertex = X.size()/3;
  vertex_weight.resize(n_vertex);
  vertex_weight.setZero();
  for(int i=0; i<n_vertex; ++i) {
    vertex_weight[i] = inball(X.block<3,1>(i*3, 0)) * 1.0;
  }
}

double UTILITY::CollisionBallForceTetFull::energy(const Eigen::VectorXd &X) const
{
  assert(X.size()%3==0);
  int n_vertex = X.size()/3;
  Eigen::VectorXd vertex_weight;
  judge(X, vertex_weight); // @TODO here can be refine
  double energy_ret = 0.0;
  for(int i=0; i<n_vertex; ++i) {
    double val = 0.0;
    Eigen::Vector3d v = X.block<3,1>(i*3,0);
    collision_ball_energy_(&val, &v[0], &_ball_center[0], &_r);
    energy_ret += _k*vertex_weight[i]*val;
  }
  return energy_ret;
}

/**
 * \partial E_collision / \partial X
 */
void UTILITY::CollisionBallForceTetFull::force(const Eigen::VectorXd &X, Eigen::VectorXd &out) const
{
  assert(X.size()%3==0);
  int n_vertex = X.size()/3;
  Eigen::VectorXd vertex_weight;
  judge(X, vertex_weight); // @TODO here can be refine
  out.resize(X.size());
  out.setZero();
  for (int i=0; i<n_vertex; ++i) {
    Eigen::Vector3d v = X.block<3,1>(i*3, 0);
    Eigen::Vector3d jac(0,0,0);
    if((v-_ball_center).squaredNorm() > EPS) {
      collision_ball_energy_jac_(&jac[0], &v[0], &_ball_center[0], &_r);
    }
    out.block<3,1>(i*3,0) = _k*vertex_weight[i]*jac;
  }
}

void push_triplets(double* mat, const int rows, const int cols,
                   const int start_row, const int start_col,
                   std::vector<Eigen::Triplet<double>>& trips) {
  double *ptr = mat;
  for(int r=0; r<rows; ++r) {
    for(int c=0; c<cols; ++c) {
      trips.push_back(Eigen::Triplet<double>(start_row+r, start_col+c, *ptr));
      ++ptr;
    }
  }
}

const Eigen::SparseMatrix<double>& UTILITY::CollisionBallForceTetFull::K(const Eigen::VectorXd &X)
{
  assert(X.size()%3==0);
  int n_vertex = X.size()/3;
  _K.resize(X.size(), X.size());
  Eigen::VectorXd vertex_weight;
  judge(X, vertex_weight); // @TODO here can be refine

  // std::cout << "vertex_weight: " << vertex_weight.transpose() << std::endl;
  std::vector<Eigen::Triplet<double>> trips;
  for (int i=0; i<n_vertex; ++i) {
    Eigen::Vector3d v = X.block<3,1>(i*3,0);
    Eigen::Matrix<double, 3, 3> hes;
    hes.setZero();
    if((v-_ball_center).squaredNorm() > EPS) {
      collision_ball_energy_hes_(&hes(0,0), &v[0], &_ball_center[0], &_r);
    }
    hes = _k*vertex_weight[i]*hes;
    push_triplets(&hes(0,0), 3,3, i*3, i*3, trips);
  }
  _K.setFromTriplets(trips.begin(), trips.end());
  return _K;
}
