#include "spring_potential.h"

#include <jtflib/mesh/mesh.h>
#include <zjucad/matrix/io.h>
#include <Eigen/Sparse>

using namespace jtf::mesh;
using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;

spring_potential::spring_potential(const matrixst &tris, const matrixd &nods)
    : tris_(tris), nods_(nods) {
    shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(tris_));
    edge_num_ = e2c->edges_.size();

    edge_.resize(2, edge_num_);
    len_.resize(edge_num_);
    stiff_ = zjucad::matrix::ones(edge_num_, 1);
    d_.resize(3 * edge_num_);

    matrixd edge_vec(3, edge_num_);
#pragma omp parallel for
    for (size_t i = 0; i < edge_num_; ++i) {
        edge_(colon(), i) = e2c->get_edge(i);
        matrixd points = nods_(colon(), edge_(colon(), i));
        edge_vec(colon(), i) = points(colon(), 0) - points(colon(), 1);
        len_[i] = norm(edge_vec(colon(), i));
    }
    std::copy(edge_vec.begin(), edge_vec.end(), d_.data());

    //construct matrix L
    for (size_t i = 0; i < edge_num_; ++i) {
        size_t pid = edge_(0, i);
        size_t qid = edge_(1, i);
        for (size_t k = 0; k < 3; ++k) {
            L_triplet_.push_back(Eigen::Triplet<double>(3 * pid + k, 3 * pid + k, stiff_[i]));
            L_triplet_.push_back(Eigen::Triplet<double>(3 * qid + k, 3 * qid + k, stiff_[i]));
            L_triplet_.push_back(Eigen::Triplet<double>(3 * pid + k, 3 * qid + k, -stiff_[i]));
            L_triplet_.push_back(Eigen::Triplet<double>(3 * qid + k, 3 * pid + k, -stiff_[i]));
        }
    }
    L_.resize(nods_.size(), nods_.size());
    L_.reserve(L_triplet_.size());
    L_.setFromTriplets(L_triplet_.begin(), L_triplet_.end());
    L_.makeCompressed();

    //construct matrix J
    for (size_t i = 0; i < edge_num_; ++i) {
        size_t pid = edge_(0, i);
        size_t qid = edge_(1, i);
        for (size_t k = 0; k < 3; ++k) {
            J_triplet_.push_back(Eigen::Triplet<double>(pid * 3 + k, i * 3 + k, stiff_[i]));
            J_triplet_.push_back(Eigen::Triplet<double>(qid * 3 + k, i * 3 + k, -stiff_[i]));
        }
    }
    J_.resize(nods_.size(), 3 * edge_num_);
    J_.reserve(J_triplet_.size());
    J_.setFromTriplets(J_triplet_.begin(), J_triplet_.end());
    J_.makeCompressed();

}

int spring_potential::update_d(const Eigen::VectorXd &nods)
{
#pragma omp parallel for
    for (size_t i = 0; i < edge_num_; ++i) {
        Eigen::VectorXd d0 = nods.segment<3>(edge_(0, i) * 3) - nods.segment<3>(edge_(1, i) * 3);
        d_.segment<3>(3 * i) = len_[i] * d0 / d0.norm();
    }
    return 0;
}

size_t spring_potential::nx(void) const { return nods_.size(); }

size_t spring_potential::nf(void) const { return 1;}

int spring_potential::eval(size_t k, const double *x, const coo2val_t<FLT, INT> &cv,
                     hj::math_func::func_ctx *ctx = 0 ) const
{
    if ( k == 0 ) {
        Eigen::VectorXd x_(nods_.size());
        std::copy(x, x + nods_.size(), x_.data());
        int32_t c[] = {0};
        cv[c] += 0.5 * x_.dot(L_ * x_) - x_.dot(J_ * d_);
    }
    if ( k == 1 ) {
        Eigen::VectorXd x_(nods_.size());
        std::copy(x, x + nods_.size(), x_.data());
        Eigen::VectorXd gra;
        gra = L_ * x_ - J_ * d_;
        for (size_t i = 0; i < gra.size(); ++i) {
            int32_t c[] = {0, i};
            cv[c] += gra[i];
        }
    }
    if ( k == 2 ) {
#pragma omp paralle for
        for (size_t j = 0; j < nods_.size(); ++j) {
            for (size_t cnt = L_.outerIndexPtr()[j]; cnt < L_.outerIndexPtr()[j + 1]; ++j) {
                size_t i = L_.innerIndexPtr()[cnt];
                int32_t c[] = {0, i, j};
                cv[c] += L_.valuePtr()[cnt];
            }
        }
    }
    return 0;
}

int spring_potential::patt(size_t k, coo_set<int_type> &cs, const coo_l2g &l2g,
                 hj::math_func::func_ctx *ctx = 0 ) const {
    if ( k == 2 ) {
#pragma omp paralle for
        for (size_t j = 0; j < nods_.size(); ++j) {
            for (size_t cnt = L_.outerIndexPtr()[j]; cnt < L_.outerIndexPtr()[j + 1]; ++j) {
                size_t i = L_.innerIndexPtr()[cnt];
                int32_t c[] = {0, i, j};
                l2g.add(cs, c);
            }
        }
    }
    return 0;
}

size_t spring_potential::nnz(size_t k) const {
    if ( k == 0 )
        return -1;
    if ( k == 1 )
        return -1;
    if ( k == 2 )
        return L_.nonZeros();
}


spring_potential *build_spring_energy(const zjucad::matrix::matrix<double> &nods,
                                      const zjucad::matrix::matrix<size_t> &tris) {
    return new spring_potential(tris, nods);
}
