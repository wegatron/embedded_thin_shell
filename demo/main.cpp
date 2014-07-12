#include <iomanip>
#include <FullStVKSimulator.h>
#include "colide_rigid_data.h"
#include "embedded_shell_simulator.h"
#include "single_point_simulator.h"

using namespace std;
using namespace COLIDE_RIGID;
using namespace ZSW;
using namespace SIMULATOR;

int InitSim(const SenceData &sdata, pSimulator sim, pSinglePointSimulator ssim, pEmbeddedShellSimulator shell_sim)
{
  return 0;
}

Vector3d JoinForce(const VectorXd &extforce)
{
  assert(extforce.size()%3==0);
  Vector3d ret(0,0,0);
  for(int i=0; extforce.size(); ++i) {
    ret[i%3] += extforce[i];
  }
  return ret;
}

string CalFileName(const string &prefix, int out_steps, int width) {
  stringstream ss;
  ss << prefix << setw(width) << setfill('0') << out_steps << ".obj";
  return ss.str();
}

int Excute(const char *inifile) {
  // load data
  SenceData sdata;
  sdata.InitDataFromFile(inifile);

  pSimulator sim = pSimulator(new FullStVKSimulator());
  pSinglePointSimulator ssim = pSinglePointSimulator(new SinglePointSimulator());
  pEmbeddedShellSimulator shell_sim = pEmbeddedShellSimulator(new EmbeddedShellSimulator());

  InitSim(sdata, sim, ssim, shell_sim);

  for (int i=0; i<sdata.steps_; ++i) {
    ssim->forward();
    sim->forward();
    for (int j=0; sdata.plans_.size(); ++j) {
      sdata.plans_[i]->Collide(sdata.tet_mesh_->nodes(), sdata.kd_, sim->getModifyFullDisp(), sim->getV());
    }
    VectorXd extforce;
    sdata.rigid_ball_.Collide(sdata.tet_mesh_->nodes(), sdata.k_, sim->getFullDisp(), extforce);
    // solid simulator set extforce
    sim->setExtForce(extforce + sdata.tet_mesh_gravity_);
    // signle point simulator set extforce
    Vector3d tmp_force = JoinForce(extforce)+sdata.rigid_ball_.CalGravity(sdata.g_);
    ssim->SetExtForce(tmp_force);

    shell_sim->forward(sim->getFullDisp());
    // output
    if (i%sdata.output_steps_ == 0) {
      static int out_steps = 0;
      sdata.rigid_ball_.ExportObj(CalFileName(sdata.out_ball_prefix_, out_steps, 4));
      ExportObj(CalFileName(sdata.out_ball_prefix_, out_steps, 4), shell_sim->GetCell(), shell_sim->GetNodes(), shell_sim->GetNormal(), true);
      ++out_steps;
    }
  }
}
