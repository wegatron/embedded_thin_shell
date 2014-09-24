#include <iostream>

#include <CollisionForceTetFull.h>

using namespace std;

template<typename val_type>
val_type maxFabsArray(const val_type *data, const int size) {
  val_type max_val = fabs(data[0]);
  for (int i=1; i<size; ++i) {
    double v = fabs(data[i]);
    max_val = (v>max_val) ? v : max_val;
  }
  return max_val;
}

double graErr(UTILITY::CollisionBallForceTetFull &collision_energy, Eigen::VectorXd &x)
{
  Eigen::VectorXd g;
  collision_energy.force(x,g);
  std::cerr << "max fabs(g):" << maxFabsArray(&g[0],g.size()) << endl;
  const double eps = 1e-6;
  for(int i=0; i<g.size(); ++i) {
    double save = x[i];
    double v[2] = {0,0};
    x[i] = save + eps;
    v[0] = collision_energy.energy(x);
    x[i] = save - eps;
    v[1] = collision_energy.energy(x);
    g[i] -= (v[0]-v[1])/(2*eps);
    x[i] = save;
  }
  return maxFabsArray(&g[0], g.size());
}

template<typename val_type>
val_type maxFabsSparseMat(const Eigen::SparseMatrix<val_type> &mat)
{
  val_type max_val = 0;
  for (int i = 0; i < mat.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(mat,i); it; ++it) {
      max_val = (it.value()>max_val) ? it.value() : max_val;
    }
  }
  return max_val;
}

double hesErr(UTILITY::CollisionBallForceTetFull &collision_energy, Eigen::VectorXd &x)
{
  const Eigen::SparseMatrix<double> &K = collision_energy.K(x);
  Eigen::MatrixXd dense_k(K);

  std::cerr << "max fabs(K):" << maxFabsSparseMat(K) << endl;
  Eigen::VectorXd g[2];
  g[0].resize(x.size()); g[1].resize(x.size());
  const double eps = 1e-6;
  for (int c=0; c<x.size(); ++c) {
    double save = x[c];
    x[c] = save + eps;
    collision_energy.force(x,g[0]);
    x[c] = save - eps;
    collision_energy.force(x,g[1]);
    dense_k.col(c) -= (g[0]-g[1])/(2*eps);
    x[c] = save;
  }
  return maxFabsArray(dense_k.data(), dense_k.size());
}

int main(int argc, char *argv[])
{
  UTILITY::CollisionBallForceTetFull collision_energy(2.5);
  collision_energy.setBallCenter(Eigen::Vector3d(0,0,0));
  collision_energy.setR(1.0);

  Eigen::VectorXd X =  Eigen::VectorXd::Random(12);
  cout << "x is: " << X.transpose() << endl;

  cout << "energy is: " << collision_energy.energy(X) << endl;
  Eigen::VectorXd f;
  collision_energy.force(X, f);
  cout << "force: " << f.transpose() << endl;

  cout << "gra_err: " << graErr(collision_energy, X) << endl;

  const Eigen::Matrix<double, -1, -1> &K = collision_energy.K(X);

  cout << "hes_err: " << hesErr(collision_energy, X);
  return 0;
}
