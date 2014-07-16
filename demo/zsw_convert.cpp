#include "zsw_convert.h"

void ZSW::Convert (const UTILITY::VVec3d &vvec3d, zjucad::matrix::matrix<double> &zju_matrixd)
{
  zju_matrixd.resize(3,vvec3d.size());
  for (int i=0; i<vvec3d.size(); ++i)  {
    zju_matrixd(0,i) = vvec3d[i][0];
    zju_matrixd(1,i) = vvec3d[i][1];
    zju_matrixd(2,i) = vvec3d[i][2];
  }
}

void ZSW::Convert (const UTILITY::VVec4i &vvec4i, zjucad::matrix::matrix<size_t> &zju_matrixi)
{
  zju_matrixi.resize(4, vvec4i.size());
  for (int i=0; i<vvec4i.size(); ++i) {
    zju_matrixi(0,i) = vvec4i[i][0];
    zju_matrixi(1,i) = vvec4i[i][1];
    zju_matrixi(2,i) = vvec4i[i][2];
    zju_matrixi(3,i) = vvec4i[i][3];
  }
}

// void ZSW::zsw_convert (const hj::sparse::spm_csc<double> &zjusp_m, zjucad::matrix::matrix<double> &zjudens_m)
// {
//   size_t col = zjusp_m.size(1);
//   size_t row = zjusp_m.size(2);
//   for (size_t j = 0; j < col; ++j)
//     for (size_t i = 0; i < row; ++i)
//      zjudens_m(i, j) = zjusp_m(i, j);
// }
