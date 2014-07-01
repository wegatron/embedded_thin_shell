#include <GL/glew.h>
#include <GL/gl.h>
#include "PassiveObject.h"
using namespace SIMULATOR;

PassiveObject::PassiveObject(pQGLViewerExt view):LocalframeManipulatoionExt(view){
  
  initial_center.setZero();
  current_center.resize(3,1);
  current_center.setZero();
  scale = 1.0f;
  collision_force_penalty = 10000.0f;
  obj = pObjmesh(new Objmesh());
  mtl.ambient[0] = 0.1f;       mtl.ambient[1] = 0.1f;       mtl.ambient[2] = 0.1f;
  mtl.diffuse[0] = 0.6f;       mtl.diffuse[1] = 0.6f;       mtl.diffuse[2] = 0.6f;
}

void PassiveObject::draw()const{

  const Vector3d translate = current_center-initial_center;
  glPushMatrix();
  glTranslated(translate[0],translate[1],translate[2]);
  glScaled(scale,scale,scale);
  drawMesh();
  glPopMatrix();
}

void PassiveObject::drawWithNames()const{

  glFlush();
  glPushName(0);

  const Vector3d translate = current_center-initial_center;
  glPushMatrix();
  glTranslated(translate[0],translate[1],translate[2]);
  glScaled(scale,scale,scale);
  drawMesh();
  glPopMatrix();

  glPopName();
}

void PassiveObject::select(const vector<int> &sel_ids){

  dragged = false;
  assert_le(sel_ids.size(),1);
  if (1 == sel_ids.size() ){
	assert_eq(sel_ids[0],0);
	dragged = true;
  }
}

void PassiveBall::collisionVelocity(pTetMesh_const tet_mesh, const VectorXd &u,VectorXd &velocity, double time_step) {

  assert_eq(u.size(), tet_mesh->nodes().size()*3);
  // assert_gt(collision_force_penalty,0.0f);
  // force.resize(u.size());
  // force.setZero();

  const VVec3d &nodes = tet_mesh->nodes();
  const int n = nodes.size();
  const double r = radius();
  for (int i = 0; i < n; ++i){

	// compute forces according to distance to the center of the ball.
	const Vector3d x = nodes[i]+u.segment<3>(i*3);
	const double diff = r - (x-current_center).norm();
        const Vector3d normal = (x-current_center)/(x-current_center).norm();
        const Vector3d vnode = velocity.block(i*3,0, 3,1); // test if we can change velocity by vnode
        const double m = tet_mesh->material()._rho[i];
        // cout << __FILE__ << __LINE__ << ":" << vnode.dot(normal) << " k_limit_:" <<k_limit_ << endl; 
	if (diff > 0) { //&& vnode.dot(normal)<k_limit_*diff/time_step) {
	  // tmpforce = diff*(x-current_center)*collision_force_penalty;
         // cout << diff << ":" << k_stiffness_ << ":" << time_step << ":" << m << endl;
         // cout << "change velocity:" << normal.transpose()*(diff*k_stiffness_*time_step)/m << endl;
          assert( !(m<1e-8 && m>-1e-8) );
          velocity.block(i*3,0, 3,1) = vnode + diff*k_stiffness_*time_step/m*normal;
          // double exceed = (velocity.block(i*3,0, 3,1)).dot(normal) - 1.1*diff/time_step;
          // if (exceed > 0)
          // {
          //   velocity.block(i*3,0, 3,1) -= exceed*normal;
          // }
	}
  }
}

void PassiveBall::collisionForce(pTetMesh_const tet_mesh, const VectorXd &u,VectorXd &velocity, VectorXd &force, double time_step, vector<int>& col_nodes){
  assert_eq(u.size(), tet_mesh->nodes().size()*3);
  assert_gt(collision_force_penalty,0.0f);
  force.resize(u.size());
  force.setZero();

  const VVec3d &nodes = tet_mesh->nodes();
  const int n = nodes.size();
  const double r = radius();
  col_nodes.clear();
  for (int i = 0; i < n; ++i){

	// compute forces according to distance to the center of the ball.
	const Vector3d x = nodes[i]+u.segment<3>(i*3);
	const double diff = r-(x-current_center).norm();

         Vector3d vnode = velocity.block(i*3,0, 3,1);
         const Vector3d normal = (x-current_center)/(x-current_center).norm();
         double vnode_value=vnode.dot(normal);
         // if (diff > 0 && vnode_value<k_limit_*diff/time_step){
         // if (diff > 0 && vnode_value<0){
         if (diff > 0){
          col_nodes.push_back(i);
	  force.segment<3>(i*3) = diff*normal*collision_force_penalty;
	}
  }
 }

void PassiveBall::collisionAdj(pTetMesh_const tet_mesh, VectorXd &u, VectorXd &v, double time_step, const vector<int> &col_nodes)
{
  const VVec3d &nodes = tet_mesh->nodes();
  const int n = col_nodes.size();
  const double r = radius();
  for (int i = 0; i < n; ++i){
    const int nodes_id = col_nodes[i];
    const Vector3d x = nodes[nodes_id]+u.segment<3>(nodes_id*3);
    const double diff = r - (x-current_center).norm();
    if(diff < -0.01*r)
      {
        const Vector3d disp_adj = (x-current_center)*(0.99*diff/(x-current_center).norm());
        u.segment<3>(nodes_id*3) += disp_adj;
        // v.segment<3>(nodes_id*3) += disp_adj/time_step;
      }
  }
}

void PassiveBall::collisionForceOri(pTetMesh_const tet_mesh, const VectorXd &u,VectorXd &force)
{
  assert_eq(u.size(), tet_mesh->nodes().size()*3);
  assert_gt(collision_force_penalty,0.0f);
  force.resize(u.size());
  force.setZero();

  const VVec3d &nodes = tet_mesh->nodes();
  const int n = nodes.size();
  const double r = radius();
  for (int i = 0; i < n; ++i){

	// compute forces according to distance to the center of the ball.
	const Vector3d x = nodes[i]+u.segment<3>(i*3);
	const double diff = r-(x-current_center).norm();
	if (diff > 0){
	  force.segment<3>(i*3) = diff*(x-current_center)/(x-current_center).norm()*collision_force_penalty;
	}
  }
}

void PassiveBall::collisionDisp(pTetMesh_const tet_mesh, VectorXd &u, VectorXd &v, double time_step)
{
  const VVec3d &nodes = tet_mesh->nodes();
  const int n = nodes.size();
  const double r = radius();
  for (int i = 0; i < n; ++i){
    const Vector3d x = nodes[i]+u.segment<3>(i*3);
    const double diff = r - (x-current_center).norm();
    if(diff >0)
      {
        const Vector3d disp_adj = (x-current_center)*(diff/(x-current_center).norm());
        u.segment<3>(i*3) += disp_adj;
        v.segment<3>(i*3) += disp_adj/time_step;
      }
  }
}
