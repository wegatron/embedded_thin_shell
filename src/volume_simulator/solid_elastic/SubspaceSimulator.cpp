#include "SubspaceSimulator.h" 
using namespace SIMULATOR;

SubspaceSimulator::SubspaceSimulator():sim_name("cubature"){

  stvkModel = pReducedElasticModel(new CubaturedElasticModel());
  simulator = pReducedSimulator(new ReducedImpLogConSimulator(stvkModel));
}

bool SubspaceSimulator::forward(){

  bool succ = simulator->forward();
  succ &= stvkModel->computeFullDisp(simulator->getQ(),full_disp);
  return succ;
}
