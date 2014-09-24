#include <iomanip>

#include <FullStVKSimulator.h>
#include <Timer.h>

#include "collide_rigid_data.h"
#include "embedded_shell_simulator.h"
#include "single_point_simulator.h"
#include "zsw_convert.h"
#include "vrml2_io.h"

using namespace std;
using namespace COLIDE_RIGID;
using namespace ZSW;
using namespace SIMULATOR;

int InitSim(const SenceData &sdata, pSimulator sim, pSinglePointSimulator ssim, pEmbeddedShellSimulator shell_sim)
{
  bool succ = sim->init(sdata.ini_file_);
  assert(succ);
  sim->setVolMesh(sdata.tet_mesh_);
  sdata.tet_mesh_->writeVTK("/home/wegatron/test.vtk");
  sim->precompute();
  sim->setExtForce(sdata.tet_mesh_gravity_);

  ssim->SetTimeStep(sdata.time_step_);
  ssim->SetQuality(sdata.rigid_ball_.GetQuality());
  ssim->CalSetGravity(sdata.g_normal_);
  ssim->SetV(sdata.ball_v_);
  cout << "init shell sim..." << endl;
  shell_sim->Init(sdata.tet_mesh_, sdata.subdivision_time_);
  cout << "shell sim init succ" << endl;
  return 0;
}

Vector3d JoinForce(const VectorXd &extforce)
{
  assert(extforce.size()%3==0);
  Vector3d ret(0,0,0);
  for(int i=0; i<extforce.size(); ++i) {
    ret[i%3] += extforce[i];
  }
  return ret;
}

string CalFileName(const string &prefix, const string &suffix, const int out_steps, const int width) {
  stringstream ss;
  ss << prefix << setw(width) << setfill('0') << out_steps << suffix;
  return ss.str();
}

int Excute(const char *inifile) {
  // load data
  SenceData sdata;
  int ret = sdata.InitDataFromFile(inifile);
  if (ret!=0) { return __LINE__; }
  pSimulator sim = pSimulator(new FullStVKSimulator()); // @todo change using FullStVKSimModelExt

  // pFullStVKSimModelExt def_model = pFullStVKSimModelExt(new FullStVKSimModelExt());
  // pSimulator sim = pSimulator(new FullStVSimulator(def_model));

  pSinglePointSimulator ssim = pSinglePointSimulator(new SinglePointSimulator());
  pEmbeddedShellSimulator shell_sim = pEmbeddedShellSimulator(new EmbeddedShellSimulator());

  InitSim(sdata, sim, ssim, shell_sim);
  vector<VectorXd> record_u;
  Timer timer2;
  Timer timer;
  timer.start();
  int out_steps = 0;

  Eigen::Vector3d c_center = sdata.rigid_ball_.GetCenter();
  const double r = sdata.rigid_ball_.GetR();
  // @todo add collision energy to sim.
  for (int i=0; i<sdata.steps_; ++i) {
    // timer.start();
    sim->forward();
    // timer.stop("solid sim time:");
    if(out_steps < 60) {
      ssim->forward();
      sdata.rigid_ball_.Transform(ssim->GetU());
      c_center += ssim->GetU();
    }

    // collide plane
    for (int j=0; j<sdata.planes_.size(); ++j) {
      sdata.planes_[j]->Collide(sdata.tet_mesh_->nodes(), sdata.soft_kd_, sim->getModifyFullDisp(), sim->getV());
      // sdata.planes_[j]->Collide(sdata.rigid_kd_, ssim->GetV(), sdata.rigid_ball_);
    }
    // VectorXd extforce;
    // sdata.rigid_ball_.Collide(sdata.tet_mesh_->nodes(), sdata.stiff_k_, sim->getFullDisp(), extforce);

    // signle point simulator set extforce
    // Vector3d join_force = JoinForce(extforce);
    // ssim->SetExtForce(-join_force);

    // output
    if (i%sdata.output_steps_ == 0) {
      timer.stop("one frame time:");
      cout << "[zsw_info]: step" << i << endl;
      VectorXd u = sim->getFullDisp();
      // shell_sim->forward(u);
      sdata.rigid_ball_.ExportVtk(CalFileName(sdata.out_ball_prefix_, ".vtk", out_steps, 4));
      // ExportObj(CalFileName(sdata.out_shell_mesh_prefix_, ".obj", out_steps, 4), shell_sim->GetCell(), shell_sim->GetNodes(), shell_sim->GetNormal(), true);
      // vector<size_t> face_vec;
      // vector<double> point_vec;
      // ZSW::Convert(shell_sim->GetCell(), face_vec);
      // ZSW::Convert(shell_sim->GetNodes(), point_vec);
      // ExportVrml2(CalFileName(sdata.out_shell_mesh_prefix_, ".vrml", out_steps, 0),face_vec, point_vec);
      record_u.push_back(u);
      ++out_steps;
      timer.start();
    }
  }
  { // save as vtk files.
    bool succ = sdata.tet_mesh_->writeVTK(sdata.out_tet_mesh_prefix_, record_u, false);
    assert(succ);
  }
}

int main(int argc, char *argv[])
{
  if (argc != 2) {
    cout << "usage: collision_demo [inifile]" << endl;
    return __LINE__;
  }
  Excute(argv[1]);
  return 0;
}
