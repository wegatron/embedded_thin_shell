#include <GL/glew.h>
#include <GL/gl.h>
#include "PassiveObject.h"
using namespace SIMULATOR;

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

void PassiveBall::collision(pTetMesh_const tet_mesh, VectorXd &coll_forces){
  
  
}
