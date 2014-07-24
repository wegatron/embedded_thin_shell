#ifndef __SPRING_POTENTIAL_H__
#define __SPRING_POTENTIAL_H__

#include <zjucad/matrix/matrix.h>
#include <hjlib/math_func/math_func.h>
#include <Eigen/Sparse>

typedef hj::math_func::math_func_t<double, int32_t> func_t;

//E(x) = 0.5 x^T Lx - x^T Jd

class spring_potential : public func_t {
    typedef zjucad::matrix::matrix<size_t> matrixst;
    typedef zjucad::matrix::matrix<double> matrixd;
    typedef double  FLT;
    typedef int32_t INT;
public  :
    spring_potential(const matrixst &tris, const matrixd &nods);
    virtual size_t nx(void) const;
    virtual size_t nf(void) const;
    virtual int    eval(size_t k, const double *x, const hj::math_func::coo2val_t<FLT, INT> &cv,
                        hj::math_func::func_ctx *ctx = 0 ) const;
    virtual int    patt(size_t k, hj::math_func::coo_set<INT> &cs, const hj::math_func::coo_l2g &l2g,
                        hj::math_func::func_ctx *ctx) const;
    virtual size_t nnz(size_t k) const;
    int            update_d(const Eigen::VectorXd  &nods);

private :
    const matrixst                  &tris_;
    const matrixd                   &nods_;

    size_t                          edge_num_;
    matrixst                        edge_;
    matrixd                         len_;
    matrixd                         stiff_;
    Eigen::VectorXd                 d_;

    Eigen::SparseMatrix<double>          L_;
    Eigen::SparseMatrix<double>          J_;
    std::vector<Eigen::Triplet<double>>  L_triplet_;
    std::vector<Eigen::Triplet<double>>  J_triplet_;
};

spring_potential *build_spring_energy(const zjucad::matrix::matrix<double> &nods,
                                      const zjucad::matrix::matrix<size_t> &tris);



#endif
