#ifndef __SHELL_LINEAR_SOLVER_H__
#define __SHELL_LINEAR_SOLVER_H__


#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <jtflib/function/function.h>
#include <zjucad/matrix/matrix.h>
#include <hjlib/sparse/sparse.h>
#include <memory>
#include <petscksp.h>
#include <mpi.h>
#include <petscversion.h>
#include <boost/property_tree/ptree.hpp>

using namespace Eigen;
using namespace std;

/**
  *           \partial E
  * f_ext - ------------- = 0;
  *           \partial x
  * when f_ext = 0, the equation becomes
  *
  * \partial E
  * ----------- = 0;
  * \partial x
  *
  * in shell deformation, the energy is nearly quadratic, so by linearizing
  * above equation and solving it, we could get a good approximation to the
  * real solution.
  *
  * */

class LinearSolver
{
public :
    LinearSolver(const jtf::function::functionN1_t<double, int32_t> &f);
    int forward(zjucad::matrix::matrix<double> &x);

protected :
    int assemble_A(zjucad::matrix::matrix<double> &x);
    int assemble_b(zjucad::matrix::matrix<double> &x);

protected :
    //model provides
    jtf::function::functionN1_t<double, int32_t> *f_;
    hj::sparse::csc<double, int32_t> H_;
    zjucad::matrix::matrix<double> g_;

    //solver needs
    SparseMatrix<double> A_;
    VectorXd b_;
    vector<Triplet<double>> tripletA_;
};

class PetscKSP
{
public :
    PetscKSP(const zjucad::matrix::matrix<double>               &x,
             const jtf::function::functionN1_t<double, int32_t> &f);
    int solve(double *x, size_t max_iter);
    virtual ~PetscKSP();
private :
    jtf::function::functionN1_t<double, int32_t> *f_;
    hj::sparse::csc<double, int32_t> H_;
    zjucad::matrix::matrix<double>   g_;

    Mat A_;
    Vec b_, X_;
    zjucad::matrix::matrix<int> ptr_, idx_;
    MPI_Comm comm_;
    KSP solver_;
    PC pc_;
    int dim_;
    const double eps_;
};

int optimizeCJ(const jtf::function::functionN1_t<double, int32_t> &f,
               zjucad::matrix::matrix<double>                     &x);

//*******************************beautiful cut line*************************************

namespace cj {

class linear_solver
{
    /**
      * standard interface of linear solver
      * can has different implementation
      */
public :
    static linear_solver *create(const hj::sparse::csc<double, int32_t> &A,
                                 boost::property_tree::ptree            &pt);
    virtual int solve(const double *b, double *x, size_t rhs, boost::property_tree::ptree &pt) = 0;
    virtual ~linear_solver() {}
};

}

#endif
