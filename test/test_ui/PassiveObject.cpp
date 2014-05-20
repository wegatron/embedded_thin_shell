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

void PassiveBall::collision(pTetMesh_const tet_mesh, const VectorXd &u,VectorXd &force){
  
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
	  force.segment<3>(i*3) = diff*(x-current_center)*collision_force_penalty;
	}
  }

}
