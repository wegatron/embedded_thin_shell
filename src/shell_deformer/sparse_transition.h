#ifndef __SPARSE_TRANSITION_H__
#define __SPARSE_TRANSITION_H__

#include <Eigen/Sparse>
#include <hjlib/sparse/sparse.h>

class sparse_transition {
public :
    int eigen2hj(Eigen::SparseMatrix<double>       &x,
                 hj::sparse::csc<double, int32_t>  &y);
    int hj2eigen(hj::sparse::csc<double, int32_t>  &x,
                 Eigen::SparseMatrix<double>       &y);
};


#endif
