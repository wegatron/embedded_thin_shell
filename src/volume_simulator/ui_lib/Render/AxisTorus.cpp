#include <WindowsHeaders.h>
#include <GL/glew.h>
#include <math.h>
#include "AxisTorus.h"
#include <QGLViewer/config.h>
using namespace QGLVEXT;

void AxisTorus::draw()const{

  float scalor[3] = {0.1f ,0.1f ,0.1f};
  if (selected_axis == 0) {
	  drawTorus(0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, scalor[0]);
  }
  else
	  drawTorus(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f ,scalor[0]);
  if (selected_axis == 1)
  {
	  drawTorus(0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f ,scalor[1]);
  }
  else
      drawTorus(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f ,scalor[1]);
  if (selected_axis == 2)
  {
	  drawTorus(1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f ,scalor[2]);
  }
  else
      drawTorus(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f ,scalor[2]);
}

void AxisTorus::drawWithNames ()const{

  float scalor[3] = {0.1f ,0.1f ,0.1f};
  glFlush();

  glPushName(0);
  drawTorus(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f ,scalor[0]);
  glPopName();

  glPushName(1);
  drawTorus(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f ,scalor[1]);
  glPopName();

  glPushName(2);
  drawTorus(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f ,scalor[2]);
  glPopName();
}

void AxisTorus::drawTorus(float x, float y, float z,
						  float r, float g, float b,
						  const float s)const{

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glColor4f(r,g,b,1.0f);
  glPushMatrix();
  glScalef(s,s,s);
  glRotatef(90.0f, x, y, z);
  
  glLineWidth(10);
  const double TWOPI = 2 * M_PI;
  const int numc = 100;
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < numc; i++)
  {
	  glNormal3d(5*cos(1.0*i*TWOPI/numc), 0.0, 5*sin(1.0*i*TWOPI/numc));
	  glVertex3d(5*cos(1.0*i*TWOPI/numc), 0.0, 5*sin(1.0*i*TWOPI/numc));
  }
  glEnd();

  glPopMatrix();
  glDisable(GL_COLOR_MATERIAL);
  glPopAttrib();
}
