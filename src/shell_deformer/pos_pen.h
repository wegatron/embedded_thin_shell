#ifndef CJ_POS_PENALTY_H_
#define CJ_POS_PENALTY_H_


#include <hjlib/math_func/math_func.h>
#include <zjucad/matrix/matrix.h>


typedef double FLT;
typedef int32_t INT;

typedef hj::math_func::math_func_t<FLT, INT> func_t;

using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;


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
        maps_.resize(3 * wgt_mat_.size());
        size_t _c = maps_.size();
        for (size_t i = 0; i < _c; ++i) {
            size_t _idx = i / 3;
            size_t _offset = i % 3;
            for (size_t j = 0; j < wgt_mat_[_idx].size(); ++j)
                maps_[i].push_back(make_pair(wgt_mat_[_idx][j].first * 3 + _offset, wgt_mat_[_idx][j].second));
        }
    }

    virtual size_t nx(void) const {
        return nodes_.size();
    }

    virtual size_t nf(void) const {
        return maps_.size();
    }

    virtual int eval(size_t k, const FLT *x, const coo2val_t<FLT, INT> &cv,
                     hj::math_func::func_ctx *ctx = 0 ) const
    {
        if ( k == 0 ) {
#pragma omp parallel for
            for (size_t i = 0; i < maps_.size(); ++i) {
                INT c[] = {i};
                for (size_t j = 0; j < maps_[i].size(); ++j) {
                    cv[c] += (x[maps_[i][j].first] - nodes_[maps_[i][j].first]) *
                            maps_[i][j].second;
                }
            }
        }
        if ( k == 1 ) {
#pragma omp parallel for
            for (size_t i = 0; i < maps_.size(); ++i) {
                for (size_t j = 0; j < maps_[i].size(); ++j) {
                    INT c[] = {i, maps_[i][j].first};
                    cv[c] += maps_[i][j].second;
                }
            }
        }
        return 0;
    }

    virtual int patt(size_t k, coo_set<INT> &cs, const coo_l2g &l2g,
                     hj::math_func::func_ctx *ctx = 0) const
    {
        if ( k == 1 ) {
            for (size_t i = 0; i < maps_.size(); ++i) {
                for (size_t j = 0; j < maps_[i].size(); ++j) {
                    int32_t c[] = {i, maps_[i][j].first};
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
            return 3 * nodes_.size(2);
    }

public :
    matrix<double> nodes_;
private :
    const std::vector<std::vector<std::pair<size_t, double>>> &wgt_mat_;
    vector<vector<pair<size_t, double>>> maps_;
};


position_difference *build_pos_pen_diff(const zjucad::matrix::matrix<double>                       &Bq,
                                        const std::vector<std::vector<std::pair<size_t, double>>>  &clt_w);





#endif
