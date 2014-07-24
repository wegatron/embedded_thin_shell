#ifndef __CJ_POS_PENALTY_H__
#define __CJ_POS_PENALTY_H__

#include <hjlib/math_func/math_func.h>
#include <zjucad/matrix/matrix.h>


#include <hjlib/util/hrclock.h>

typedef double FLT;
typedef int32_t INT;

typedef hj::math_func::math_func_t<FLT, INT> func_t;

using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;


typedef struct map_tuple{
   size_t val_idx_;
   size_t var_idx_;
   double w_;
   map_tuple() {}
   map_tuple(const size_t val_idx, const size_t var_idx, const double w)
       : val_idx_(val_idx), var_idx_(var_idx), w_(w) {}
}map_tuple;


//class position_difference : public func_t
//{
//public :
//    position_difference(const zjucad::matrix::matrix<double>       &Bq,
//                        const vector<vector<pair<size_t, double>>> &clt_w)
//        : nodes_(Bq), clt_w_(clt_w) {
//    }

//    virtual size_t nx(void) const {
//        return nodes_.size();
//    }

//    virtual size_t nf(void) const {
//        return nodes_.size(1) * clt_w_.size();
//    }

//    virtual int eval(size_t k, const FLT *x, const coo2val_t<FLT, INT> &cv,
//                     hj::math_func::func_ctx *ctx = 0 ) const
//    {
//        if ( k == 0 ) {
//            for (size_t i = 0; i < 3 * clt_w_.size(); ++i) {
//                INT c[] = {i};
//                for (size_t j = 0; j < clt_w_[i / 3].size(); ++j) {
//                    size_t idx = clt_w_[i / 3][j].first;
//                    cv[c] += (x[idx * 3 + i % 3] - nodes_[idx * 3 + i % 3]) *
//                            clt_w_[i / 3][j].second;
//                }
//            }
//        }
//        if ( k == 1 ) {
//            for (size_t i = 0; i < 3 * clt_w_.size(); ++i) {
//                for (size_t j = 0; j < clt_w_[i / 3].size(); ++j) {
//                    size_t idx = clt_w_[i / 3][j].first;
//                    INT c[] = {i, idx * 3 + i % 3};
//                    cv[c] += clt_w_[i / 3][j].second;
//                }
//            }
//        }
//        return 0;
//    }

//    virtual int patt(size_t k, coo_set<INT> &cs, const coo_l2g &l2g,
//                     hj::math_func::func_ctx *ctx = 0) const
//    {
//        if ( k == 1 ) {
//            for (size_t i = 0; i < 3 * clt_w_.size(); ++i) {
//                for (size_t j = 0; j < clt_w_[i / 3].size(); ++j) {
//                    size_t idx = clt_w_[i / 3][j].first;
//                    int32_t c[] = {i, idx * 3 + i % 3};
//                    l2g.add(cs, c);
//                }
//            }
//        }
//        return 0;
//    }

//    virtual size_t nnz(size_t k) const {
//        if ( k == 0 )
//            return -1;
//        if ( k == 1 )
//            return 3 * nodes_.size(2);
//    }

//public :
//    matrix<double> nodes_;
//private :
//    const std::vector<std::vector<std::pair<size_t, double>>> &wgt_mat_;
//};



class position_difference : public func_t
{
public :
    position_difference(const zjucad::matrix::matrix<double>       &Bq,
                        const vector<vector<pair<size_t, double>>> &wgt_mat)
        : nodes_(Bq), wgt_mat_(wgt_mat) {
        for (size_t i = 0; i < wgt_mat_.size(); ++i) {
            for (size_t j = 0; j < wgt_mat_[i].size(); ++j) {
                for (size_t k = 0; k < 3; ++k) {
                    size_t val_idx = i * 3 + k;
                    size_t var_idx = wgt_mat_[i][j].first * 3 + k;
                    double coeff = wgt_mat_[i][j].second;
                    temp_maps_.push_back(map_tuple(val_idx, var_idx, coeff));
                }
            }
        }
        maps_.resize(temp_maps_.size());
        std::copy(temp_maps_.begin(), temp_maps_.end(), maps_.begin());
    }

    virtual size_t nx(void) const {
        return nodes_.size();
    }

    virtual size_t nf(void) const {
        return wgt_mat_.size() * 3;
    }

    virtual int eval(size_t k, const FLT *x, const coo2val_t<FLT, INT> &cv,
                     hj::math_func::func_ctx *ctx) const
    {
        if ( k == 0 ) {
#pragma omp parallel for
            for (size_t i = 0; i < maps_.size(); ++i) {
                INT c[] = {maps_[i].val_idx_};
                cv[c] += ( x[maps_[i].var_idx_] - nodes_[maps_[i].var_idx_] ) * maps_[i].w_;
            }
        }
        if ( k == 1 ) {
#pragma omp parallel for
            for (size_t i = 0; i < maps_.size(); ++i) {
                INT c[] = {maps_[i].val_idx_, maps_[i].var_idx_};
                cv[c] += maps_[i].w_;
            }
        }
        return 0;
    }

    virtual int patt(size_t k, coo_set<INT> &cs, const coo_l2g &l2g,
                     func_ctx *ctx) const
    {
        if ( k == 1 ) {
            for (size_t i = 0; i < maps_.size(); ++i) {
                INT c[] = {maps_[i].val_idx_, maps_[i].var_idx_};
                l2g.add(cs, c);
            }
        }
        return 0;
    }

    virtual size_t nnz(size_t k) const {
        if ( k == 0 )
            return -1;
        if ( k == 1 )
            return 3 * nodes_.size(2);
    }

public :
    matrix<double> nodes_;
protected :
    const std::vector<std::vector<std::pair<size_t, double>>> &wgt_mat_;
    std::vector<map_tuple> temp_maps_;
    matrix<map_tuple> maps_;

};

position_difference *build_pos_pen_diff(const zjucad::matrix::matrix<double>                       &Bq,
                                        const std::vector<std::vector<std::pair<size_t, double>>>  &clt_w);


#endif
