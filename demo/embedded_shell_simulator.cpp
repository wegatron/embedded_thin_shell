#include "embedded_shell_simulator.h"

#include <Objmesh.h>

#include "zsw_convert.h"

using namespace ZSW;

void EmbeddedShellSimulator::Init (const UTILITY::pTetMesh &tet_mesh, const int subdivision_time) {
  const UTILITY::VVec3d &nodes = tet_mesh->nodes();
  const UTILITY::VVec4i &tets = tet_mesh->tets();
  zjucad::matrix::matrix<size_t> tet_cell(4, tets.size());
  tet_nodes_.resize(3, nodes.size());
  ZSW::Convert(tets, tet_cell);
  ZSW::Convert(nodes, tet_nodes_);
  gen_outside_shell(tet_cell, tet_nodes_, shell_cell_, shell_nodes_, shell_normal_, subdivision_time, __EMBED_DEPTH);

  size_t row = tet_nodes_.size(2);
  size_t col = shell_nodes_.size(2);
  hj::sparse::spm_csc<double> B(row, col);
  tet_embed(tet_nodes_, tet_cell, shell_nodes_, B);
  B_.resize(row, col);
  for (size_t j = 0; j < col; ++j)
    for (size_t i = 0; i < row; ++i)
     B_(i, j) = B(i, j);

  cluster_machine handle(shell_cell_, shell_nodes_, __CLUSTER_RADIUS);
  handle.partition(__REGION_COUNT);
  shell_deformer_ = pShellDeformer(new deformer(shell_cell_, shell_nodes_, shell_nodes_, handle.regions_));
}

void EmbeddedShellSimulator::forward (const VectorXd &tet_disp) {
  assert(tet_disp.size()%3 == 0);
  size_t col = tet_disp.size()/3;
  matrix<double> dx(3, col),
    xq(shell_nodes_.size(1), shell_nodes_.size(2));
  std::copy(&tet_disp[0], &tet_disp[0]+tet_disp.size(), &dx(0,0));
  xq = (tet_nodes_ + dx) * B_;
  shell_deformer_->deform(shell_nodes_, xq);
}

int ZSW::ExportObj (const string filename, const matrixt &shell_cell, const matrixd &shell_nodes,
               const matrixd &shell_normal, const bool gen_normal) {
  VectorXd shell_nodes_e(shell_nodes.size(2)*3);
  VectorXd shell_normal_e(shell_nodes.size(2)*3);
  VectorXi shell_cell_e(shell_cell.size(2)*3);
  cout << "shell nodes:" << shell_nodes.size(1) << ":" << shell_nodes.size(2) << endl;
  for (int i=0; i<shell_nodes.size(2); ++i) {
    shell_nodes_e[i*3] = shell_nodes(0,i);
    shell_nodes_e[i*3+1] = shell_nodes(1,i);
    shell_nodes_e[i*3+2] = shell_nodes(2,i);

    shell_normal_e[i*3] = shell_normal(0,i);
    shell_normal_e[i*3+1] = shell_normal(1,i);
    shell_normal_e[i*3+2] = shell_normal(2,i);
  }
  for (int i=0; i<shell_cell.size(2); ++i) {
    shell_cell_e[i*3] = shell_cell(0,i);
    shell_cell_e[i*3+1] = shell_cell(1,i);
    shell_cell_e[i*3+2] = shell_cell(2,i);
  }
  UTILITY::Objmesh obj_mesh(shell_nodes_e, shell_cell_e);
  obj_mesh.setVertNormals(shell_normal_e);
  obj_mesh.write(filename);
  return 0;
}
