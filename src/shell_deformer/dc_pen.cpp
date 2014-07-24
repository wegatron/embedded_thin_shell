#include "dc_pen.h"

#include <hjlib/math_func/operation.h>

using namespace std;
using namespace hj::math_func;
using namespace zjucad::matrix;

position_difference_ *build_dc_pen_diff_(size_t n, const matrixi_t &idx, const matrixf_t &pos, double weight)
{
  return new position_difference_(n, idx, pos, weight);
}
