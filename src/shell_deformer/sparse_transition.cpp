#include "sparse_transition.h"

int sparse_transition::eigen2hj(Eigen::SparseMatrix<double>       &x,
                                hj::sparse::csc<double, int32_t>  &y)
{
    x.makeCompressed();
    y.resize(x.rows(), x.cols(), x.nonZeros());
    y.idx().resize(x.nonZeros());
    y.ptr().resize(x.cols() + 1);
    y.val().resize(x.nonZeros());

    std::copy(x.innerIndexPtr(), x.innerIndexPtr() + x.nonZeros(), y.idx().begin());
    std::copy(x.outerIndexPtr(), x.outerIndexPtr() + x.cols() + 1, y.ptr().begin());
    std::copy(x.valuePtr(), x.valuePtr() + x.nonZeros(), y.val().begin());
    return 0;
}

int sparse_transition::hj2eigen(hj::sparse::csc<double, int32_t>  &x,
                                Eigen::SparseMatrix<double>       &y)
{
    y.resize(x.size(1), x.size(2));
    y.reserve(nnz(x));
    std::copy(x.idx().begin(), x.idx().end(), y.innerIndexPtr());
    std::copy(x.ptr().begin(), x.ptr().end(), y.outerIndexPtr());
    std::copy(x.val().begin(), x.val().end(), y.valuePtr());
    return 0;
}
