#ifndef CJ_INTERPOLATOR_H_
#define CJ_INTERPOLATOR_H_

#include <zjucad/matrix/matrix.h>
#include <hjlib/sparse/sparse.h>


int gen_outside_shell(const zjucad::matrix::matrix<size_t>   &tet_mesh,
                      const zjucad::matrix::matrix<double>   &tet_nodes,
                      zjucad::matrix::matrix<size_t>         &shell,
                      zjucad::matrix::matrix<double>         &shell_nodes,
                      zjucad::matrix::matrix<double>         &shell_normal,
                      const size_t                           sub_time = 1,
                      const double                           embed_dist = 0.025);




int tet_embed(const zjucad::matrix::matrix<double>    &v,
              const zjucad::matrix::matrix<size_t>    &tet,
              const zjucad::matrix::matrix<double>    &pts,
              hj::sparse::spm_csc<double>             &coef);






























#endif
