#ifndef __QUAD_BENDING_H__
#define __QUAD_BENDING_H__

#include <zjucad/matrix/matrix.h>
#include <hjlib/math_func/math_func.h>

using namespace std;
using namespace zjucad::matrix;

typedef double FLT;
typedef int32_t INT;
typedef hj::math_func::math_func_t<FLT, INT> func_t;

func_t *build_quad_bending_energy(const matrix<double> &nods, const matrix<size_t> &tris);

#endif
