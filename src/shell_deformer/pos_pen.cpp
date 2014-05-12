#include "pos_pen.h"

#include <hjlib/math_func/operation.h>
#include <vector>

using namespace std;
using namespace hj::math_func;
using namespace zjucad::matrix;

//matrix<double> position_difference::nodes_ = zjucad::matrix::zeros(1, 1);

position_difference *build_pos_pen_diff(const zjucad::matrix::matrix<double>        &Bq,
                                        const vector<vector<pair<size_t, double>>>  &clt_w)
{
    return new position_difference(Bq, clt_w);
}


