#ifndef _FULLSTVKSIMULATOR_H_
#define _FULLSTVKSIMULATOR_H_

#include "Simulator.h"
#include <ConMatrixTools.h>
#include <FullSimulator.h>

#include <FullElasticModel.h>

#include "ElasticModel/CollisionForceTetFull.h"

using namespace UTILITY;

namespace SIMULATOR{
  
  /**
   * @class FullStVKSimulator solid simulation using full stvk model.
   * 
   */
  class FullStVKSimulator:public Simulator{
	
  public:
	FullStVKSimulator(){
	  stvkModel = pFullStVKSimExtModel(new FullStVKSimExtModel());
          #ifdef IMPLICIT_SOLID_SIMULATOR
	  simulator = pLagImpFullSim(new LagImpFullSim(stvkModel));
          #else
          simulator = pPenSemiImpFullSim (new PenSemiImpFullSim (stvkModel));
          #endif
	}
	string name()const{
	  return "full stvk";
	}
	bool init(const string filename){
	  return simulator->init(filename);
	}
	void setVolMesh(pTetMesh_const tetMesh){
	  stvkModel->setTetMesh(tetMesh);
	}
	bool precompute(){
	  return simulator->prepare();
	}
	void reset(){
	  clearExtForce();
	  removeAllConNodes();
	  simulator->reset();
	}

	void setConNodes(const vector<int> &nodes){
          cout << "set con nodes!!!!!!!!!!!!!!!!" << endl;
	  if (nodes.size() <= 0){
		removeAllConNodes();
		return;
	  }

	  const int n = stvkModel->dimension()/3;
	  if (n > 0){
		VecT trip_C;
		UTILITY::computeConM(nodes, trip_C, n);
		simulator->setConM(trip_C, nodes.size()*3, n*3);
	  }
	}
	void setUc(const VectorXd &uc){
	  simulator->setUc(uc);
	}
	void removeAllConNodes(){
	  simulator->removeAllCon();
	}
	
	void setExtForceOfNode(const int nodeId,const double f[3]){
	  simulator->setExtForceOfNode(f, nodeId);
	}
	void setExtForce(const VectorXd &f_ext){
	  simulator->setExtForce(f_ext);
	}
	void clearExtForce(){
	  simulator->setExtForceForAllNodes(0.0f,0.0f,0.0f);
	}

	bool forward(){
	  return simulator->forward();
	}

        void setCollisionEnergy(pCollisionForceTetFull collide_energy) {
          _collide_energy = collide_energy;
          stvkModel->setColideEnergy(_collide_energy);
        }

	const VectorXd &getFullDisp()const{
	  return simulator->getU();
	}

        VectorXd &getModifyFullDisp() const {
          return simulator->getModifyU();
        }

        VectorXd& getV() const {
          return simulator->getVelocity();
        }
        
        double getTimestep() const
        {
          return simulator->getTimestep();
        }
	bool computeElasticForce(const VectorXd &u,VectorXd &f)const{
	  if (stvkModel)
		return stvkModel->evaluateF(u,f);
	  return false;
	}
	
  private:
	pFullStVKSimExtModel stvkModel;
        pCollisionForceTetFull _collide_energy;
        pBaseFullSim simulator;
	/* pLagImpFullSim simulator; */
        /* pPenSemiImpFullSim simulator; */
  };
  
  typedef boost::shared_ptr<FullStVKSimulator> pFullStVKSimulator;
  
}//end of namespace

#endif /*_FULLSTVKSIMULATOR_H_*/
