#include <Timer.h>
#include <FullStVKSimulator.h>

using namespace UTILITY;
using namespace SIMULATOR;


void collision_plane(const VVec3d &nodes, VectorXd &v, VectorXd &u, double kd, double plane_z,
                    bool up)
{
  for (int i=0; i<nodes.size(); ++i)
    {
      if (up)
        {
          if (nodes[i][2] + u[i*3+2] > plane_z)
            {
              u[i*3+2] = plane_z - nodes[i][2];
              v[i*3+2] = -kd*v[i*3+2];
            }
        }
      else
        {
            if (nodes[i][2] + u[i*3+2] < plane_z)
            {
              u[i*3+2] = plane_z - nodes[i][2];
              v[i*3+2] = -kd*v[i*3+2];
            }
        }
    }
}


void case_one()
{
  // open init json file
  const string ini_file = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/simple_cube/simu_full.ini";
  JsonFilePaser jsonf;
  bool succ = jsonf.open(ini_file);
  assert(succ);

  pTetMesh tet_mesh = pTetMesh(new TetMesh());
  { // init tetrahedron mesh.
    string vol_file;
    succ = jsonf.readFilePath("vol_file",vol_file);
    assert(succ);
    succ = tet_mesh->load(vol_file);
    assert(succ);

    string mtl_file;
    jsonf.readFilePath("elastic_mtl",mtl_file);
    succ = tet_mesh->loadElasticMtl(mtl_file);
    assert(succ);
  }

  pSimulator simulator = pSimulator(new FullStVKSimulator());
  { // init simulator
    succ = simulator->init(ini_file);
    assert(succ);
    simulator->setVolMesh(tet_mesh);
    simulator->precompute();
  }

  { // load and set fixed nodes
    vector<int> nodes;
    succ = UTILITY::loadVec("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/simple_cube/model/con_nodes.bou", nodes,UTILITY::TEXT);
    assert(succ);
    cout << "num of fixed nodes: " << nodes.size() << endl;
    simulator->setConNodes(nodes);
    VectorXd uc(nodes.size()*3);
    uc.setZero();
    simulator->setUc(uc);
  }

  // simulation for 200 steps, record displacements
  vector<VectorXd> record_u;
  { // set external forces, then simulate 50 steps
    const double f[3] = {0, 0, 700000};
    simulator->setExtForceOfNode(0, f);
    simulator->setExtForceOfNode(1, f);
    simulator->setExtForceOfNode(2, f);
    simulator->setExtForceOfNode(3, f);

    for (int i = 0; i < 800; ++i){
      cout << "step " << i << endl;
      simulator->forward();
      VectorXd &v = simulator->getV();
      VectorXd &u = simulator->getModifyFullDisp();
      collision_plane(tet_mesh->nodes(), v, u, 0.6, 1.8, true);
      record_u.push_back(simulator->getFullDisp());
    }
  }

  { // save as vtk files.
    succ = tet_mesh->writeVTK("/home/wegatron/tempt/test_collision", record_u);
    if (!succ)
      {
        cout << "error output" << endl;
      }
    assert(succ);
  }

  cout << "[INFO]DONE!\n";

}

void case_two()
{
  // open init json file
  const string ini_file = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/simple_cube/simu_full.ini";
  JsonFilePaser jsonf;
  bool succ = jsonf.open(ini_file);
  assert(succ);

  pTetMesh tet_mesh = pTetMesh(new TetMesh());
  { // init tetrahedron mesh.
    string vol_file;
    succ = jsonf.readFilePath("vol_file",vol_file);
    assert(succ);
    succ = tet_mesh->load(vol_file);
    assert(succ);

    string mtl_file;
    jsonf.readFilePath("elastic_mtl",mtl_file);
    succ = tet_mesh->loadElasticMtl(mtl_file);
    assert(succ);
  }

  pSimulator simulator = pSimulator(new FullStVKSimulator());
  { // init simulator
    succ = simulator->init(ini_file);
    assert(succ);
    simulator->setVolMesh(tet_mesh);
    simulator->precompute();
  }

  { // load and set fixed nodes
    vector<int> nodes;
    succ = UTILITY::loadVec("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/simple_cube/model/con_nodes2.bou", nodes,UTILITY::TEXT);
    assert(succ);
    cout << "num of fixed nodes: " << nodes.size() << endl;
    simulator->setConNodes(nodes);
    VectorXd uc(nodes.size()*3);
    uc.setZero();
    simulator->setUc(uc);
  }

  // simulation for 200 steps, record displacements
  vector<VectorXd> record_u;
  { // set external forces, then simulate 50 steps
    const double f[3] = {0, 0, -800000};
    simulator->setExtForceOfNode(0, f);
    simulator->setExtForceOfNode(1, f);
    simulator->setExtForceOfNode(2, f);
    simulator->setExtForceOfNode(3, f);

    for (int i = 0; i < 800; ++i){
      cout << "step " << i << endl;
      simulator->forward();
      VectorXd &v = simulator->getV();
      VectorXd &u = simulator->getModifyFullDisp();
      collision_plane(tet_mesh->nodes(), v, u, 0.6, -0.2, false);
      record_u.push_back(simulator->getFullDisp());
    }
  }
  
  { // save as vtk files.
    succ = tet_mesh->writeVTK("/home/wegatron/tempt/test_collision", record_u);
    if (!succ)
      {
        cout << "error output" << endl;
      }
    assert(succ);
  }

  cout << "[INFO]DONE!\n";

}

// a simple example demonstrate the usage of the solid simulator.
int main(int argc, char *argv[]){
  case_two();
  return 0;
}
