#ifndef _LAGIMPFULLSIM_H_
#define _LAGIMPFULLSIM_H_

#include <boost/shared_ptr.hpp>
#include <BaseFullSim.h>
#include <JsonFilePaser.h>

namespace SIMULATOR{

  typedef vector<Eigen::Triplet<double> > VecT;
  
  /**
   * @class LagImpFullSim simulator in full space using Lagrangian constraints
   * and implicit integration scheme.
   * @see Doc/Latex/IntegrationMethods.pdf
   */
  class LagImpFullSim: public BaseFullSim{
	
  public:
	LagImpFullSim(pBaseFullModel def_model);
	bool prepare(){
	  bool succ = false;
	  if (BaseFullSim::prepare()){
		succ = def_model->evaluateM(M);
	  }
	  C_Ct_triplet.clear();
	  resetM_triplet();
	  return succ;
	}
	void setTimeStep(const double h){
	  BaseFullSim::setTimeStep(h);
	  resetM_triplet();
	}
	void setDampings(const double alpha_k,const double alpha_m){
	  BaseFullSim::setDampings(alpha_k, alpha_m);
	  resetM_triplet();
	}
	void setConM(const VecT &C_triplet,const int C_rows,const int C_cols);
	void removeAllCon(){
	  BaseFullSim::removeAllCon();
	  C_Ct_triplet.clear();
	  C.resize(0,0);
	}

	// simulation
	bool forward();

  protected:
	bool assembleA();
	bool assembleB();
	bool resetM_triplet();
	bool resetK_triplet();
	void resetA_triplet();
	void copyTriplet(VecT &Full_triplet, const VecT &sub_triplet, const int start)const;
	
  private:
	VectorXd f; // internal forces.
	SparseMatrix<double> M; // mass matrix.
	SparseMatrix<double> C; // constraint matrix.
	
	/**
	 * right hand side and left hand side for the linear system at each time
	 * step to sovle for the integration, where
	 *     |H C^t|
	 * A = |C  O |, b = M*v + h*(fext-f),
	 * and H = (1+h*alpha_m)*M + h*(h+alpha_k)*K(u),
	 * C is the constraint matrix.
	 */
	SparseMatrix<double> A; 
	VectorXd b;  

	/**
	 * Inorder to fastly assemble sparse matrix A, we record the triplets for
	 * each submatrix in A_triplet, then generate A from A_triplet each step.
	 * A_triplet is consisted of three parts: 
	 * 1. scaled mass matrix: update when intialization or time step is changed.
	 * 2. constraint matrix and its transpose, update when constraints is changed.
	 * 3. scaled stiffness matrix, update at each step.
	 */
	VecT C_Ct_triplet;     // C and C^t.
	VecT Scaled_M_triplet; // (1+alpha_m)*M.
	VecT Scaled_K_triplet; // h*(h+alpha_k)*K(u).
	VecT A_triplet; // [scaled_M, C, C^T, scaled_K].
  };
  
  typedef boost::shared_ptr<LagImpFullSim> pLagImpFullSim;
  
}//end of namespace

#endif /*_LAGIMPFULLSIM_H_*/
