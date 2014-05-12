#include <stdio.h>
#include <string.h>
#include <assertext.h>
#include <Log.h>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/UmfPackSupport>
#include "LagImpFullSim.h"
using namespace SIMULATOR;

LagImpFullSim::LagImpFullSim(pBaseFullModel def_model):
  BaseFullSim(def_model){}

void LagImpFullSim::setConM(const VecT &C_triplet,const int C_rows,const int C_cols){

  // initialize C
  assert_ge(C_rows,0);
  assert_ge(C_cols,0);
  C.resize(C_rows, C_cols);
  C.reserve(C_triplet.size());
  if (C_triplet.size() > 0){
	C.setFromTriplets(C_triplet.begin(), C_triplet.end());
  }

  // initialize C_Ct_triplet
  const int n = getDim();
  C_Ct_triplet.clear();
  C_Ct_triplet.reserve(C_triplet.size()*2);
  for (size_t i = 0; i < C_triplet.size(); ++i){

	const int r = C_triplet[i].row() + n;
	const int c = C_triplet[i].col();
	const double v = C_triplet[i].value();
	C_Ct_triplet.push_back( Eigen::Triplet<double>(r,c,v) ); // C
	C_Ct_triplet.push_back( Eigen::Triplet<double>(c,r,v) ); // C^t
  }
}

bool LagImpFullSim::forward(){

  // assemble A and b
  bool succ = assembleA();
  succ &= assembleB();
  assert_eq(A.rows(),A.cols());
  assert_eq(A.rows(),b.size());

  // solve equation
  if (succ){

	UmfPackLU<SparseMatrix<double> > solver;
	solver.compute(A);
	if(solver.info()!=Success) {
	  ERROR_LOG("decomposition failed");
	  return false;
	}
	v = solver.solve(b);
	if(solver.info()!=Success) {
	  ERROR_LOG("solving failed");
	  return false;
	}
  }
  if (succ){
	u = u + h*v.head(getDim());
  }
  return succ;
}

bool LagImpFullSim::assembleA(){

  bool succ = resetK_triplet();
  if (succ){
	resetA_triplet();
	// create sparse matrix A from triplet.
	A.resize(M.rows()+C.rows(), M.cols() + C.rows());
	A.reserve(A_triplet.size());
	A.setFromTriplets( A_triplet.begin(), A_triplet.end() );
  }
  A.makeCompressed();
  return succ;
}

bool LagImpFullSim::assembleB(){

  const int n3 = this->getDim();
  const int len_c = uc.size();
  b.resize(n3 + len_c);

  if (len_c > 0){
	assert_gt (h,0.0f);
  	b.tail(len_c) = (1.0f/h)*(uc - C*u);
  }

  bool succ = def_model->evaluateF(u,f);
  if (succ){
	assert_eq (M.rows(), n3);
  	b.head(n3) = M*v.head(n3) + h*(fext-f);
  }
  return succ;
}

void LagImpFullSim::resetA_triplet(){
  
  const size_t len = Scaled_M_triplet.size()+Scaled_K_triplet.size()+C_Ct_triplet.size();

  if (A_triplet.capacity() < len){
	A_triplet.reserve((len*4)/3);
  }
  A_triplet.resize(len);
  copyTriplet(A_triplet, Scaled_M_triplet, 0);
  copyTriplet(A_triplet, C_Ct_triplet, Scaled_M_triplet.size());
  copyTriplet(A_triplet, Scaled_K_triplet, Scaled_M_triplet.size() + C_Ct_triplet.size());
}

bool LagImpFullSim::resetM_triplet(){

  assert (def_model != NULL);
  bool succ = true;
  if( def_model->evaluateM_triplet(Scaled_M_triplet) ){

	//scale to (1+h*alpha_m)*M.
	for (size_t i = 0; i < Scaled_M_triplet.size(); ++i){
	  const int r = Scaled_M_triplet[i].row();
	  const int c = Scaled_M_triplet[i].col();
	  const double v = Scaled_M_triplet[i].value()*(1.0f+h*alpha_m);
	  Scaled_M_triplet[i]= Eigen::Triplet<double>(r,c,v);
	}  
  }else{
	ERROR_LOG("Failed to compute the triplet for the mass matrix.");
	succ = false;
  }
  return succ;
}

bool LagImpFullSim::resetK_triplet(){

  Scaled_K_triplet.clear();  
  bool succ = def_model->evaluateK_triplet(u, Scaled_K_triplet);
  if (succ){
	// scale the stiffness matrix to h*(h+alpha_k)*K(u)
	for (size_t i = 0; i < Scaled_K_triplet.size(); ++i){
	  const int r = Scaled_K_triplet[i].row();
	  const int c = Scaled_K_triplet[i].col();
	  const double v = Scaled_K_triplet[i].value()*h*(h+alpha_k);
	  Scaled_K_triplet[i]= Eigen::Triplet<double>(r,c,v);
	}
  }
  return succ;
}

void LagImpFullSim::copyTriplet(VecT &Full_triplet, const VecT &sub_triplet, const int start)const{

  assert_ge (Full_triplet.size(), sub_triplet.size() + start);
  if (sub_triplet.size() > 0){

	const size_t len = sub_triplet.size()*sizeof(Eigen::Triplet<double>);
	const void* src = &(sub_triplet[0]);
	void* target = &(Full_triplet[start]);
	memcpy(target, src, len);
  }
}
