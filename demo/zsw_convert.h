#ifndef ZSW_CONVERT_H
#define ZSW_CONVERT_H

#include <vector>
#include <Eigen/Dense>
#include <zjucad/matrix/matrix.h>

#include <TetMesh.h>

namespace ZSW{
void Convert (const UTILITY::VVec3d &vvec3d, zjucad::matrix::matrix<double> &zju_matrixd);
void Convert (const UTILITY::VVec4i &vvec4i, zjucad::matrix::matrix<size_t> &zju_matrixi);
//void Convert (const hj::sparse::spm_csc<double> &zjusp_m, zjucad::matrix::matrix<double> &zjudens_m);
}

#endif /* ZSW_CONVERT_H */
