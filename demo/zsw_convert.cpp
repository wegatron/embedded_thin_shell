#include "zsw_convert.h"

#include <iostream>

using namespace std;

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

void ZSW::Convert (const zjucad::matrix::matrix<double> &zju_matrixd, UTILITY::VVec3d &vvec3d)
{
  size_t col = zju_matrixd.size(2);
  vvec3d.reserve(col);
  vvec3d.clear();
  for (size_t i=0; i<col; ++i)  {
    Eigen::Vector3d tmp_vec(zju_matrixd(0,i), zju_matrixd(1,i), zju_matrixd(2,i));
    vvec3d.push_back(tmp_vec);
  }
}

void ZSW::Convert (const zjucad::matrix::matrix<double> &zju_matrixd, std::vector<double> &vecd)
{
  vecd.resize(zju_matrixd.size(1)*zju_matrixd.size(2));
  std::copy(zju_matrixd.begin(), zju_matrixd.end(), vecd.begin());
}

void ZSW::Convert (const zjucad::matrix::matrix<size_t> &zju_matrixi, UTILITY::VVec4i &vvec4i)
{
  size_t col = zju_matrixi.size(2);
  vvec4i.reserve(col);
  vvec4i.clear();

  for (int i=0; i<vvec4i.size(); ++i) {
    Eigen::Vector4i tmp_vec(zju_matrixi(0,i), zju_matrixi(1,i), zju_matrixi(2,i), zju_matrixi(3,i));
    vvec4i.push_back(tmp_vec);
  }
}

void ZSW::Convert (const zjucad::matrix::matrix<size_t> &zju_matrixi, std::vector<size_t> &veci)
{
  veci.resize(zju_matrixi.size(1)*zju_matrixi.size(2));
  std::copy(zju_matrixi.begin(), zju_matrixi.end(), veci.begin());
}

// void ZSW::zsw_convert (const hj::sparse::spm_csc<double> &zjusp_m, zjucad::matrix::matrix<double> &zjudens_m)
// {
//   size_t col = zjusp_m.size(1);
//   size_t row = zjusp_m.size(2);
//   for (size_t j = 0; j < col; ++j)
//     for (size_t i = 0; i < row; ++i)
//      zjudens_m(i, j) = zjusp_m(i, j);
// }
