#ifndef _EMBEDDED_SHELL_SIMULATOR_H_
#define _EMBEDDED_SHELL_SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <zjucad/matrix/matrix.h>
#include <TetMesh.h>

#include "interpolator/interpolator.h"
#include "shell_deformer/cluster.h"
#include "shell_deformer/deformer.h"
#include "conf_para.h"

namespace ZSW {
  typedef zjucad::matrix::matrix<size_t> matrixt;
  typedef zjucad::matrix::matrix<double> matrixd;
  typedef boost::shared_ptr< hj::sparse::spm_csc<double> > pspmcsc;
  typedef boost::shared_ptr< deformer > pShellDeformer;
  class EmbeddedShellSimulator
  {
  public:
    void Init (const UTILITY::pTetMesh &tet_mesh);
    void forward (const VectorXd &tet_disp);
    matrixt &GetCell () { return shell_cell_; }
    matrixd &GetNodes () { return shell_nodes_; }
    matrixd &GetNormal () { return shell_normal_; }
  private:
    pShellDeformer shell_deformer_;
    zjucad::matrix::matrix<size_t> shell_cell_;
    zjucad::matrix::matrix<double> shell_nodes_;
    zjucad::matrix::matrix<double> shell_normal_;
    pspmcsc B_;
  };
  typedef boost::shared_ptr<EmbeddedShellSimulator> pEmbeddedShellSimulator;

  int ExportObj (const string filename, const matrixt &shell_cell, const matrixd &shell_nodes,
                 const matrixd &shell_normal, const bool gen_normal);
}

#endif /* _EMBEDDED_SHELL_SIMULATOR_H_ */
