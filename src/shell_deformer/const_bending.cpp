#include "const_bending.h"

#include <hjlib/math_func/operation.h>
#include <hjlib/math_func/math_func.h>
#include <jtflib/mesh/mesh.h>
#include <zjucad/matrix/operation.h>
#include <zjucad/matrix/io.h>
#include "lap_operator.h"

using namespace jtf::mesh;
using namespace hj::math_func;

// E(x) = 0.5||Lx||^2
// L_ : laplacian operator used for boundless triangle mesh

class quad_bending_energy : public func_t
{
public :
    quad_bending_energy(const matrixst &tris, const matrixd &nods)
        : tris_(tris), nods_(nods) {
       p_lap_.reset(new laplacian_operator(tris_, nods_));
       p_lap_->construct_lap_matrix(L_);
       LT_ = L_.transpose();
       LTL_ = LT_ * L_;
    }

    virtual size_t nx(void) const { return nods_.size(); }

    virtual size_t nf(void) const { return 1; }


    virtual int eval(size_t k, const double *x, const coo2val_t<FLT, INT> &cv,
                     hj::math_func::func_ctx *ctx = 0) const
    {
        if ( k == 0 ) {
            VectorXd x_(nods_.size(), 1);
            std::copy(x, x + nods_.size(), x_.data());
            int32_t c[] = {0};
            cv[c] += 0.5 * x_.dot(LTL_ * x_);
        }
        if ( k == 1 ) {
            VectorXd x_(nods_.size(), 1);
            std::copy(x, x + nods_.size(), x_.data());
            VectorXd gra;
            gra = LTL_ * x_;
            for (size_t i = 0; i < gra.size(); ++i) {
                int32_t c[] = {0, i};
                cv[c] += gra[i];
            }
        }
        if ( k == 2 ) {
#pragma omp parallel for
            for (size_t j = 0; j < nods_.size(); ++j) {
                for (size_t cnt = LTL_.outerIndexPtr()[j]; cnt < LTL_.outerIndexPtr()[j + 1]; ++cnt) {
                    size_t i = LTL_.innerIndexPtr()[cnt];
                    int32_t c[] = {0, i, j};
                    cv[c] += LTL_.valuePtr()[cnt];
                }
            }
        }
        return 0;
    }

    virtual int patt(size_t k, coo_set<int_type> &cs, const coo_l2g &l2g,
                     hj::math_func::func_ctx *ctx = 0) const {
        if ( k == 2 ) {
            for (size_t j = 0; j < nods_.size(); ++j) {
                for (size_t cnt = LTL_.outerIndexPtr()[j]; cnt < LTL_.outerIndexPtr()[j + 1]; ++cnt) {
                    size_t i = LTL_.innerIndexPtr()[cnt];
                    int32_t c[] = {0, i, j};
                    l2g.add(cs, c);
                }
            }
        }
        return 0;
    }

    virtual size_t nnz(size_t k) const {
        if ( k == 0 )
            return -1;
        if ( k == 1 )
            return -1;
        if ( k == 2 )
            return LTL_.nonZeros();
    }

private :
    const matrixst                    &tris_;
    const matrixd                     &nods_;
    Eigen::SparseMatrix<double>       L_;
    Eigen::SparseMatrix<double>       LT_;
    Eigen::SparseMatrix<double>       LTL_;
    shared_ptr<laplacian_operator>    p_lap_;
};

typedef std::vector<std::shared_ptr<hj::math_func::math_func>> funcs_t;

func_t *build_quad_bending_energy(const matrix<double>  &nods,
                                  const matrix<size_t>  &tris)
{
    return new quad_bending_energy(tris, nods);
}

