#include <Objmesh.h>

void export_obj (string filename, matrix<size_t> shell_cell, matrix<double> shell_nodes, matrix<double> shell_normal)
{
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
}

void export_vtk (string filename, matrix<size_t> shell_cell, matrix<double> shell_nodes, matrix<double> shell_normal)
{
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
  obj_mesh.writeVTK(filename);
}
