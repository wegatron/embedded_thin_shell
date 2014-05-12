#include "SparseGenEigenSolver.h"
#include <memory.h>
#include <stdlib.h>
#include <arpack++/arlsmat.h>
#include <arpack++/arlgsym.h>

template <typename T>
bool EIGEN3EXT::eigensym( const T *valA,int nnzA,  const int *irowA,  const int *pcolA,
						 const T *valB,int nnzB,  const int *irowB,  const int *pcolB,
						 int n,int n_eig,T *vals,T *vecs){
  
  //check parameters
  const bool valid = (valA!=NULL&&irowA!=NULL&&pcolA!=NULL&&
				valB!=NULL&&irowB!=NULL&&pcolB!=NULL&&
				n_eig>0&&n_eig<n&&vals!=NULL&&vecs!=NULL);
  if(!valid){
	return false;
  }

  ARluSymMatrix<T> A(n, nnzA, const_cast<T*>(valA), const_cast<int*>(irowA), const_cast<int*>(pcolA));
  ARluSymMatrix<T> B(n, nnzB, const_cast<T*>(valB), const_cast<int*>(irowB), const_cast<int*>(pcolB));

  /**
   * \todo it is seems that the result is not the smallest ones when used to
   * solve the eigen-problem of MA.What is the means of these parameters?
   * 
   * ARluSymGenEig<T> dprob(n_eig, A, B,(char*)"LM"); Defining what we need: the
   * n_eig eigenvectors with smallest magnitude.
   */
  ARluSymGenEig<T> dprob('S',n_eig, A, B,0.0f,(char*)"LM");

  // Finding eigenvalues and eigenvectors.
  int coveraged = dprob.EigenValVectors(vecs,vals);
  return (coveraged==n_eig);
  return false;
  
}

template bool EIGEN3EXT::eigensym( const double *valA,int nnzA,  const int *irowA,  const int *pcolA,
								  const double *valB,int nnzB,  const int *irowB,  const int *pcolB,
								  int n,int n_eig,double *vals,double *vecs);

template bool EIGEN3EXT::eigensym( const float *valA,int nnzA,  const int *irowA,  const int *pcolA,
								  const float *valB,int nnzB,  const int *irowB,  const int *pcolB,
								  int n,int n_eig,float *vals,float *vecs);
