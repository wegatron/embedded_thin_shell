#include "linear_solver.h"

#include <iostream>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/UmfPackSupport>

using namespace std;
using namespace zjucad::matrix;

LinearSolver::LinearSolver(const jtf::function::functionN1_t<double, int32_t> &f)
{
    cout << "[INFO] initialize a linear solver\n";
    f_ = &f;
    g_ = zjucad::matrix::zeros<double>(f_->dim(), 1);
}


int LinearSolver::forward(matrix<double> &x)
{
    if ( assemble_A(x) )
        return __LINE__;
    cout << "[INFO] assemble A done\n";
    if ( assemble_b(x) )
        return __LINE__;
    cout << "[INFO] assemble b done\n";

    VectorXd X;
    X.resize(x.size());
    std::copy(x.begin(), x.end(), X.data());

    UmfPackLU<SparseMatrix<double>> solver;
    solver.compute(A_);
    if ( solver.info() != Success ) {
        cerr << "[INFO] decomposition failed\n";
        return __LINE__;
    }
    VectorXd delta = solver.solve(b_);
    if ( solver.info() != Success ) {
        cerr << "[INFO] solving falied\n";
        return __LINE__;
    }
    X += delta;

    std::copy(X.data(), X.data() + X.size(), x.begin());
    return 0;
}


int LinearSolver::assemble_A(matrix<double> &x)
{
    size_t dim = f_->dim();
    size_t hes_nnz = 0;
    size_t format = -1;
    if ( nnz(H_) == 0 ) {
        f_->hes(&x[0], hes_nnz, format, 0, 0, 0);
        H_.resize(dim, dim, hes_nnz);
        f_->hes(&x[0], hes_nnz, format, 0, &H_.ptr()[0], &H_.idx()[0]);
    }
    else
        hes_nnz = nnz(H_);
    H_.val()(colon()) = 0;
    f_->hes(&x[0], hes_nnz, format, &H_.val()[0], &H_.ptr()[0], &H_.idx()[0]);

    /// put H_ into triplet A;
    tripletA_.resize(hes_nnz);
    int cnt = 0;
    for (size_t col = 0; col < dim; ++col) {
        for (size_t p = H_.ptr()[col]; p < H_.ptr()[col + 1]; ++p) {
            size_t row = H_.idx()[p];
            tripletA_[cnt++] = Eigen::Triplet<double>(row, col, H_.val()[p]);
        }
    }

    A_.resize(dim, dim);
    A_.reserve(tripletA_.size());
    A_.setFromTriplets(tripletA_.begin(), tripletA_.end());
    A_.makeCompressed();

    return 0;
}

int LinearSolver::assemble_b(matrix<double> &x)
{
    g_(colon()) = 0;
    f_->gra(&x[0], &g_[0]);
    g_ *= -1;
    b_.resize(g_.size());
    std::copy(g_.begin(), g_.end(), b_.data());
    return 0;
}


PetscKSP::PetscKSP(const zjucad::matrix::matrix<double>               &x,
                   const jtf::function::functionN1_t<double, int32_t> &f)
    : eps_(1e-7)
{
    f_ = const_cast<jtf::function::functionN1_t<double, int32_t>*>(&f);
    dim_ = f_->dim();

    /* calculate -gra */
    g_ = zeros<double>(dim_, 1);
    f_->gra(&x[0], &g_[0]);
    g_ *= -1;

    /* calculate hes */
    size_t hes_nnz = 0;
    size_t format = -1;
    if ( nnz(H_) == 0 ) {
        f_->hes(&x[0], hes_nnz, format, 0, 0, 0);
        H_.resize(dim_, dim_, hes_nnz);
        f_->hes(&x[0], hes_nnz, format, 0, &H_.ptr()[0], &H_.idx()[0]);
    }
    else
        hes_nnz = nnz(H_);
    H_.val()(colon()) = 0;
    f_->hes(&x[0], hes_nnz, format, &H_.val()[0], &H_.ptr()[0], &H_.idx()[0]);

    comm_ = MPI_COMM_SELF;

    /// @brief Assemble A
    if ( sizeof(int32_t) == sizeof(int) ) {
        MatCreateSeqAIJWithArrays(comm_, H_.size(2), H_.size(1), &(H_.ptr()[0]),
                                         &(H_.idx()[0]), &(H_.val()[0]), &A_);
    } else {
        ptr_ = H_.ptr();
        idx_ = H_.idx();
        MatCreateSeqAIJWithArrays(comm_, H_.size(2), H_.size(1),
                                  &ptr_[0], &idx_[0], &(H_.val()[0]), &A_);
    }

    KSPCreate(comm_, &solver_);
    KSPSetOperators(solver_, A_, A_, SAME_PRECONDITIONER);

    KSPGetPC(solver_, &pc_);
    PCSetType(pc_, PCLU);
    KSPSetFromOptions(solver_);
//    KSPSetType(solver_, KSPMINRES);

    KSPSetInitialGuessNonzero(solver_, PETSC_TRUE);
    KSPSetUp(solver_);

    /// @brief Assemble b
#if ( PETSC_VERSION_MAJOR == 3 && PETSC_VERSION_MINOR == 4 )
    VecCreateSeqWithArray(comm_, 1, dim_, &g_[0], &b_);
#else
    VecCreateSeqWithArray(comm_, dim_, &g_[0], &b_);
#endif
}


int PetscKSP::solve(double *x, size_t max_iter)
{
    KSPSetTolerances(solver_, eps_, PETSC_DEFAULT, PETSC_DEFAULT, max_iter);

#if ( PETSC_VERSION_MAJOR == 3 && PETSC_VERSION_MINOR == 4 )
    VecCreateSeqWithArray(comm_, 1, dim_, x, &X_);
#else
    VecCreateSeqWithArray(comm_, dim_, x, &X_);
#endif

    PetscErrorCode ierr = KSPSolve(solver_, b_, X_); CHKERRQ(ierr);

#if ( PETSC_VERSION_MAJOR == 3 && PETSC_VERSION_MINOR <= 2)
    VecDestroy(b_);
    VecDestroy(X_);
#else
    VecDestroy(&b_);
    VecDestroy(&X_);
#endif

    return 0;
}

PetscKSP::~PetscKSP()
{
#if ( PETSC_VERSION_MAJOR == 3 && PETSC_VERSION_MINOR <= 2)
    MatDestroy(A_);
    KSPDestroy(solver_);
#else
    MatDestroy(&A_);
    KSPDestroy(&solver_);
#endif
}


class PetscInit {
public :
    PetscInit() {
        if ( !PetscInitializeCalled )
            PetscInitializeNoArguments();
    }
    virtual ~PetscInit() {
        if ( !PetscFinalizeCalled )
            PetscFinalize();
    }
};

int optimizeCJ(const jtf::function::functionN1_t<double, int32_t> &f,
               zjucad::matrix::matrix<double>                     &x)
{
//#define __EIGEN_UMFPACK_SOLVER__
#ifdef __EIGEN_UMFPACK_SOLVER__
    LinearSolver solver(f);
    cout << "[INFO] solving by Umfpack\n";
    int succ = solver.forward(x);
    if ( succ == 0 )
        cout << "[INFO] solve success\n";
    else
        cout << "[INFO] solve fail\n";
    return succ;
#else
    static PetscInit wrapper;
    PetscKSP slv(x, f);
    cout << "[INFO] solving by Petsc\n";
    int succ = slv.solve(&x[0], 10000);
    if ( succ == 0 )
        cout << "[INFO] solve success\n";
    else
        cout << "[INFO] solve fail\n";
    return succ;
#endif
}

/*======================================================================================================*/

namespace cj {

class PetscCGSolver : public linear_solver
{
public:
    PetscCGSolver(const hj::sparse::csc<double, int32_t> &A, const char *pc_str) {
        cout << "[INFO] Petsc CG method\n";
        comm = MPI_COMM_SELF;
        Dim = A.size(1);
        if ( sizeof(int32_t) == sizeof(int) ) {
            MatCreateSeqAIJWithArrays(comm, A.size(1), A.size(2),
                                      const_cast<int*>   (&A.ptr()[0]),
                                      const_cast<int*>   (&A.idx()[0]),
                                      const_cast<double*>(&A.val()[0]), &A_);
        } else {
            ptr_ = A.ptr();
            idx_ = A.idx();
            MatCreateSeqAIJWithArrays(comm, A.size(1), A.size(2),
                                      &ptr_[0], &idx_[0], const_cast<double *>(&A.val()[0]), &A_);
        }
        KSPCreate(comm, &solver);
        KSPSetOperators(solver, A_, A_, SAME_PRECONDITIONER);
        KSPGetPC(solver,&pc);
        PCSetType(pc, pc_str);
        KSPSetType(solver, KSPCG);
        KSPCGSetType(solver, KSP_CG_SYMMETRIC);
        KSPSetInitialGuessNonzero(solver,PETSC_TRUE);
        KSPSetUp(solver);
    }

    int solve(const double *b, double *x, size_t rhs, boost::property_tree::ptree &opts) {
        KSPSetTolerances(solver,PETSC_DEFAULT, PETSC_DEFAULT, PETSC_DEFAULT, 25);
        VecCreateSeqWithArray(comm, Dim, b, &B_);
        VecCreateSeqWithArray(comm, Dim, x, &X_);
        KSPSolve(solver, B_, X_);
//        int its;
//        KSPGetIterationNumber(solver,&its);
//        cout << "[INFO] iteration times : " << its << endl;
        VecDestroy(B_);
        VecDestroy(X_);
    }

    ~PetscCGSolver() {
        MatDestroy(A_);
        KSPDestroy(solver);
    }
private:
    Mat A_;
    Vec X_, B_;
    matrix<int> ptr_, idx_;
    KSP solver;
    PC  pc;
    MPI_Comm comm;
    int Dim;
};

class JacobiSolver : public linear_solver
{
public :
    JacobiSolver(const hj::sparse::csc<double, int32_t> &A, const size_t max_iter)
        : A_(A), max_iter_(max_iter) {
        cout << "[INFO] Jacobi iterative method.\n";
        x_.resize(A_.size(2), 1);
    }

    int solve(const double *b, double *x, size_t rhs, boost::property_tree::ptree &pt) {
        size_t count = max_iter_;
        while ( count-- ) {
#pragma omp parallel for
            for (size_t i = 0; i < x_.size(); ++i) {
                double sum = 0;
                for (size_t ptr = A_.ptr()[i]; ptr < A_.ptr()[i + 1]; ++ptr) {
                    sum += A_.val()[ptr] * x[A_.idx()[ptr]];
                }
                x_[i] = ( b[i] - sum ) / A_(i, i) + x[i];
            }
            std::copy(x_.begin(), x_.end(), &x[0]);
            //check if convergence is reached
        }
        return 0;
    }
private :
    matrix<double> x_;                             //store x_k+1
    const size_t max_iter_;                        //maximum of iterations
    const hj::sparse::csc<double, int32_t> &A_;    //A_ is a symmetric matrix, A^T = A;
};

linear_solver* linear_solver::create(const hj::sparse::csc<double, int32_t> &A,
                                     boost::property_tree::ptree            &opts)
{

    const string type = opts.get<string>("linear_solver/type.value", "PETsc");
    if ( type == "PETsc" ) {
        static PetscInit wrapper;
        return new PetscCGSolver(A, opts.get<string>("PETsc/pc.value", "sor").c_str());
    }
    else if ( type == "Jacobi" ) {
        return new JacobiSolver(A, 150);
    }
    else {
        cout << "[INFO] no linear solver!\n";
        return nullptr;
    }
}

}


