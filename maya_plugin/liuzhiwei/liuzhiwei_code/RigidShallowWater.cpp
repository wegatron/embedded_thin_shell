#include "RigidShallowWater.hpp"

USE_PRJ_NAMESPACE;

RigidShallowWater::RigidShallowWater(const Real size1,
                                     const Real size2,
                                     const SizeType resolution1,
                                     const SizeType resolution2,
                                     const Real gravity,
                                     const Real dt,
                                     const Real maxWHeight,
                                     const Real maxGHeight,
                                     const Real parRadius,
                                     const Real Cdeposit,
                                     const bool benchmark
                                     )
  : ShallowWater(size1,size2,resolution1,resolution2,gravity,dt,
                 maxWHeight,maxGHeight,parRadius,Cdeposit),
    RigidBodys(gravity, dt), _benchmark(benchmark),
    _CD(1000.0f),_CL(500.0f),_omega(0.8f),
    _lambda(1.0f),_Cdis(1.0f),_Cadapt(0.2f),
    _vThreshold(0.2), _alphaRn(0.86), _alphaP(0.23), _alphaN(0.01), _alphaR(0.61)
{
  ///< set static variable in particle class
  Particle::setCdeposit(Cdeposit);
  Particle::setAcceleration(Real3(0, -gravity, 0));
}

RigidShallowWater::~RigidShallowWater()
{
}

void RigidShallowWater::timestep()
{
  if(_benchmark)
  {
    RUN_BENCHMARK("1.Shallow water step time:",ShallowWater::timestep());
    RUN_BENCHMARK("2.Rigidbody step time:", RigidBodys::timestep());
    RUN_BENCHMARK("3.Couple solid with fluid time:", coupleSolidWithFluid());
    RUN_BENCHMARK("4.Generate particles time:", ShallowWater::generateParticles());
    RUN_BENCHMARK("5.Simulate all spray time:", simulateAllSpray());
	RUN_BENCHMARK("5.Simulate all splash time:", simulateAllSplash());
    RUN_BENCHMARK("6.Simulate all foams time:", simulateAllFoams());
  }
  else
  {
      ///< Height field fluid simulation 
      ShallowWater::timestep();

      ///< Solids simulation(use bullet)
      RigidBodys::timestep();
  
      ///< Two-way coupling of height field and solids
      coupleSolidWithFluid();

      ///< Particles generation 
      ShallowWater::generateParticles();
    
      ///< Spray simulation
      simulateAllSpray();  

	  ///< Spray simulation
      simulateAllSplash(); 

      ///< Foams simulation  
      simulateAllFoams();

  }
}

void RigidShallowWater::simulateAllSpray()
{
  for(std::list<Particle>::iterator ite = _spray.begin(); ite != _spray.end(); ++ite)
  {
    ite->particleFly(ShallowWater::_dt);
    Real3 pos = ite->getPosition();
    Real hfHeight = _hf->getSample(Real2(pos.x(), pos.z()));
    if(pos.y() <= hfHeight)
    {
      ite->setLifeTime(0);
      particleToHeightField(*ite);
      ite = _spray.erase(ite);
      --ite;
    }
  }
}

void RigidShallowWater::simulateAllSplash()
{
  for(std::list<Particle>::iterator ite = _splash.begin(); ite != _splash.end(); ++ite)
  {
    ite->particleFly(ShallowWater::_dt);
    Real3 pos = ite->getPosition();
    Real hfHeight = _hf->getSample(Real2(pos.x(), pos.z()));
    if(pos.y() <= hfHeight)
    {
      ite->setLifeTime(0);
      particleToHeightField(*ite);
      ite = _splash.erase(ite);
      --ite;
    }
  }
}

void RigidShallowWater::simulateAllFoams()
{
  for(std::list<Particle>::iterator ite = _foams.begin(); ite != _foams.end(); ++ite)
  {
    ite->foamAdvect(ShallowWater::_dt);
    
    if(ite->getLifeTime() <= 0)
    {
      ite = _foams.erase(ite);
      --ite;
    }
    else
    {
      ///< get the position of particle
      Real3 pos = ite->getPosition();
      ///< get the position in x-z plane
      Real2 loc(pos.x(), pos.z());

      Real hfVal = _hf->getSample(loc);
      pos.y() = hfVal;

      Real3 VF(_uf->getSample(loc), 0.0f, _wf->getSample(loc));
      VF.y() = Real2(VF.x(),VF.z()).dot(_hf->getDSample(loc));

      ///< update the position and velocity according to current position
      ite->setPosition(pos);
      ite->setVelocity(VF);      
    }
      
  }
}
void RigidShallowWater::dumpAllSpray(std::vector<Real3>& sprayPos,
                                     std::vector<Real>& sprayRadius,
                                     std::vector<Real3>& sprayVelocity) const
{
  sprayPos.resize(0);
  sprayRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _spray.begin(); ite != _spray.end(); ++ite)
  {
    Real3 pos;
    Real r;
    Real3 v;
    ite->dumpParticleToBuffer(pos, r, v);
    sprayPos.push_back(pos);
    sprayRadius.push_back(r);
    sprayVelocity.push_back(v);
  }
}

void RigidShallowWater::dumpAllSplash(std::vector<Real3>& splashPos,
                                      std::vector<Real>& splashRadius,
                                      std::vector<Real3>& splashVelocity) const
{
  splashPos.resize(0);
  splashRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _splash.begin(); ite != _splash.end(); ++ite)
  {
    Real3 pos;
    Real r;
    Real3 v;
    ite->dumpParticleToBuffer(pos, r, v);
    splashPos.push_back(pos);
    splashRadius.push_back(r);
    splashVelocity.push_back(v);
  }
}

void RigidShallowWater::dumpAllFoams(std::vector<Real3>& foamsPos,
                                     std::vector<Real>& foamsRadius) const
{
  foamsPos.resize(0);
  foamsRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _foams.begin();ite != _foams.end();++ite)
  {
    Real3 pos;
    Real r;
    ite->dumpFoamToBuffer(pos, r);
    foamsPos.push_back(pos);
    foamsRadius.push_back(r);
  }
}


/*

*Summary: Translate dumping spray to VRML2 files.

*Parameters:

*     sprayPos: A vector contains all spray's position.

*     sprayRadius: A vector contains all spray's Radius.

*     sprayVelocity: A vector contains all spray's velocity.

*Return: void

*/

void RigidShallowWater::dumpAllSprayToVRML(std::vector<Real3>& sprayPos,
                                     std::vector<Real>& sprayRadius,
                                     std::vector<Real3>& sprayVelocity) const
{
  FILE *file_wrl;
  char file_name[512];
 // float baseX = (100 / _resolution1) * (_resolution1 - 1) / 2;
  sprintf(file_name,"VRMLFrame%d.wrl",_frameNum);
  if ((file_wrl = fopen(file_name,"a"))==NULL)//creat dest file(**.obj)
  {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

  sprayPos.resize(0);
  sprayRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _spray.begin(); ite != _spray.end(); ++ite)
  {
    Real3 pos;
    Real r;
    Real3 v;
    ite->dumpParticleToBuffer(pos, r, v);

	if(ite == _spray.begin())
	{
		 fprintf(file_wrl,"Transform {\n");
	     fprintf(file_wrl,"translation  %.1f %.1f %.1f\n",pos.x(),pos.y(),pos.z());
	     fprintf(file_wrl,"children [\n");
	     fprintf(file_wrl,"DEF particle Shape{\n");
         fprintf(file_wrl,"appearance Appearance	{\n");
         fprintf(file_wrl,"material Material {\n");
	     fprintf(file_wrl,"diffuseColor 0.0 0.0 1.0\n");
	     fprintf(file_wrl," }\n");
	     fprintf(file_wrl," }\n");
	     fprintf(file_wrl,"geometry Sphere {\n");
	     fprintf(file_wrl,"  radius %.1f\n",r);
	     fprintf(file_wrl," }\n");
	     fprintf(file_wrl," }\n");
	     fprintf(file_wrl," ]\n");
	     fprintf(file_wrl," }\n\n");
	}
	else
	{
		 fprintf(file_wrl,"Transform {\n");
	     fprintf(file_wrl,"translation  %.1f %.1f %.1f\n",pos.x(),pos.y(),pos.z());
	     fprintf(file_wrl,"children [ USE particle ]\n");
		 fprintf(file_wrl,"}");

	}

	
   
   // sprayPos.push_back(pos);
   //sprayRadius.push_back(r);
   //sprayVelocity.push_back(v);
	

  }
  	fclose(file_wrl);
}
/*

*Summary: Translate dumping splash to VRML2 files.

*Parameters:

*     sprayPos: A vector contains all splash's position.

*     sprayRadius: A vector contains all splash's Radius.

*     sprayVelocity: A vector contains all splash's velocity.

*Return: void

*/
void RigidShallowWater::dumpAllSplashToVRML(std::vector<Real3>& splashPos,
                                      std::vector<Real>& splashRadius,
                                      std::vector<Real3>& splashVelocity) const
{
  FILE *file_wrl;
  char file_name[512];
 // float baseX = (100 / _resolution1) * (_resolution1 - 1) / 2;
  sprintf(file_name,"VRMLFrame%d.wrl",_frameNum);
  if ((file_wrl = fopen(file_name,"a"))==NULL)//creat dest file(**.obj)
  {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

  splashPos.resize(0);
  splashRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _splash.begin(); ite != _splash.end(); ++ite)
  {
    Real3 pos;
    Real r;
    Real3 v;
    ite->dumpParticleToBuffer(pos, r, v);
    //splashPos.push_back(pos);
    //splashRadius.push_back(r);
    //splashVelocity.push_back(v);

		 fprintf(file_wrl,"Transform {\n");
	     fprintf(file_wrl,"translation  %.1f %.1f %.1f\n",pos.x(),pos.y(),pos.z());
	     fprintf(file_wrl,"children [ USE particle ]\n");
		 fprintf(file_wrl,"}");

	
  }
  fclose(file_wrl);
}
/*

*Summary: Translate dumping foams to VRML2 files.

*Parameters:

*     sprayPos: A vector contains all foams' position.

*     sprayRadius: A vector contains all foams' Radius.

*Return: void

*/
void RigidShallowWater::dumpAllFoamsToVRML(std::vector<Real3>& foamsPos,
                                     std::vector<Real>& foamsRadius) const
{
  FILE *file_wrl;
  char file_name[512];
 // float baseX = (100 / _resolution1) * (_resolution1 - 1) / 2;
  sprintf(file_name,"VRMLFrame%d.wrl",_frameNum);
  if ((file_wrl = fopen(file_name,"a"))==NULL)//creat dest file(**.obj)
  {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

  foamsPos.resize(0);
  foamsRadius.resize(0);
  for(std::list<Particle>::const_iterator ite = _foams.begin();ite != _foams.end();++ite)
  {
    Real3 pos;
    Real r;
    ite->dumpFoamToBuffer(pos, r);
    //foamsPos.push_back(pos);
    //foamsRadius.push_back(r);
	
		 fprintf(file_wrl,"Transform {\n");
	     fprintf(file_wrl,"translation  %.1f %.1f %.1f\n",pos.x(),pos.y(),pos.z());
	     fprintf(file_wrl,"children [ USE particle ]\n");
		 fprintf(file_wrl,"}");

	  
  }
  fclose(file_wrl);
}

void RigidShallowWater::coupleSolidWithFluid()
{
  int nr = _bodies.size();
  for(int i = 0; i < nr; i++)
  {
    btRigidBody* body = _bodies[i];
    btTransform& t = body->getWorldTransform();
    //IndexedMeshArray& arr = _trimeshesForPhy[i]->getIndexedMeshArray();
    //for(int j = 0;j < arr.size(); j++)

    applyForceOnSoildAndFluid(body, t, _objMeshes[i]);
  }
}

void RigidShallowWater::applyForceOnSoildAndFluid(btRigidBody* bt, btTransform& t, WaveFrontObjMesh& m)
{
  Real3 O(t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z());
  
  Real3 force;
  
  std::vector<Real3> vertices;
  
  for ( SizeType v = 0; v < m.getVertexSize(); v++ )
  {
    btVector3 vs(m.getPoint(0, v).x(), m.getPoint(0, v).y(), m.getPoint(0, v).z());

    btVector3 vsOut = t * vs;

    //push back transformed position
    Real3 vsOutE(vsOut.x(),vsOut.y(),vsOut.z());

    vertices.push_back(vsOutE);
  }
  
  for( SizeType f = 0; f < m.getNumTriangles(); ++f )
  {
    WaveFrontObjMesh::Triangle tri = m.getTriangle(0, f);

    //calculate P, MU and VRel
    Eigen::Matrix<unsigned int, 1, 3> idx(tri.findex[0], tri.findex[1], tri.findex[2]);

    Real3 P = (vertices[idx.x()]+vertices[idx.y()]+vertices[idx.z()])/3.0f;
      
    Real3 N = (vertices[idx.y()]-vertices[idx.x()]).cross(vertices[idx.z()]-vertices[idx.x()]);
    
    Real A = N.norm()/2.0f;
    N /= (A*2.0f);

    btVector3 Vbt = bt->getVelocityInLocalPoint(t.inverse()*btVector3(P.x(),P.y(),P.z()));

    Real3 V(Vbt.x(), Vbt.y(), Vbt.z());

    Real2 Pxz(P.x(),P.z());
    
    Real3 VF(_uf->getSample(Pxz), 0.0f, _wf->getSample(Pxz));
    VF.y() = Real2(VF.x(),VF.z()).dot(_hf->getDSample(Pxz));
    
    Real3 VRel = V - VF;

    Real VRelN = std::max(VRel.norm(),Real(1E-3f));

    //calculate force on solid
    force = applyFacetForceOnSolid(P, N, A, V, VRel);

    //INFO("Force:%f,%f,%f", force.x(), force.y(), force.z());

    
    //Real3 relPos = P - O;
    //bt->applyForce(btVector3(force.x(), force.y(), force.z()),
    //               btVector3(relPos.x(),relPos.y(),relPos.z()));

    bt->applyCentralForce(btVector3(force.x(), force.y(), force.z()));
    
    //calculate force on fluid
    applyFacetForceOnFluid(P, N, A, V, VRel);

    Real2 yBB = yBoundingBox(vertices[idx.x()], vertices[idx.y()], vertices[idx.z()]);
    
    if(V.norm() > _vThreshold && is_sweep_hf(P, yBB, N, VRel))
      generateParticles(bt, vertices[idx.x()],vertices[idx.y()],vertices[idx.z()],
                        N, A);
  }
  //INFO("Force:%f,%f,%f", force.x(), force.y(), force.z());
}

void RigidShallowWater::generateParticles(const btRigidBody* bt, const Real3& v1, const Real3& v2, const Real3& v3, const Real3& N,const Real& A)
{
  ///< particle const coefficients
  const Real numProportion = 1.0;
  Real particleDragCo = 0.15;
  
  std::vector<Real3> newPosData;
  subdivision(v1, v2, v3, A, _parRadius, newPosData);

  Real3 ns = N;    

  SizeType triNum = newPosData.size() / 3;

  const btTransform& t = bt->getWorldTransform();  

  for(SizeType i = 0; i < triNum; i+=3)
  {
    Real3 ps = (newPosData[i] + newPosData[i+1] + newPosData[i+2]) / 3.0;

    btVector3 Vbt = bt->getVelocityInLocalPoint(t.inverse()*btVector3(ps.x(),ps.y(),ps.z()));
    
    Real3 vs(Vbt.x(), Vbt.y(), Vbt.z());
    
    Real2 Pxz(ps.x(),ps.z());
    Real3 Vfluid(_uf->getSample(Pxz), 0.0f, _wf->getSample(Pxz));
    Vfluid.y()=Real2(Vfluid.x(),Vfluid.z()).dot(_hf->getDSample(Pxz));

    Real3 Vrel = vs - Vfluid;
    Real2 yBB = yBoundingBox(newPosData[i], newPosData[i+1], newPosData[i+2]);
    if(is_sweep_hf(ps, yBB, ns, Vrel))
    {
      Real3 Vrn =  Vrel.dot(ns)*ns;
      Real3 Vrp = Vrel - Vrn;
      Real3 Vpar = Vfluid+_alphaRn*Vrn+_alphaP*Vrp+_alphaN*ns*fabs(Vrel.y())+
                   _alphaR*(Vrel-2*Vrel.y()*Real3(0,1,0));

      //std::srand(std::time(NULL));
      
      SizeType numPar = (SizeType) (numProportion * (vs.norm() - _vThreshold));
      for(SizeType q = 0; q < numPar; ++q)
      {
        Real3 pos = ps + 2*_parRadius*q*ns;

        jitter_position(pos, _parRadius);

        Vpar *= -1;
        
        Particle par(pos, Vpar, _parRadius, particleDragCo);
        _spray.push_back(par);
      }

    }
  }
  
}

void RigidShallowWater::applyFacetForceOnFluid(const Real3& P,const Real3& N,const Real A,const Real3& V,const Real3& VRel)
{

  int num_substeps = std::max<int>(1,(int)(Real2(V.x(),V.z()).norm() *
                                         ShallowWater::_dt/_dx + 0.5f));

  Real Vdisp = N.dot(VRel) * A * ShallowWater::_dt;
  int sign = (N.y() > 0.0f) ? 1 : -1;
  for(int q = 1;q <= num_substeps;q++)
  {
    Real3 Ps = P + V * (ShallowWater::_dt*q/num_substeps);
    int i, j;
    
    if(!_hf->getId(Real2(Ps.x(),Ps.z()), i, j)) {
      continue;
    }
    
    Real depth = _hf->get(i,j) - P.y();
    if(depth > 0.0f)
    {
      Real decay = std::exp(_lambda*(-depth));
      static int flag = 1;
      Real hfVal = _hf->get(i,j) + decay*(Vdisp/(num_substeps*_dx*_dx))*_Cdis;
      _hf->set(i, j, hfVal);

      Real coeff = std::min<Real>(1.0, decay*_Cadapt*(depth/_hf->get(i,j))*sign*
                            (ShallowWater::_dt/(_dx*_dx))*A);
      
      Real ufVal = _uf->get(i,j) + coeff*(V.x()-_uf->get(i,j));
      _uf->set(i, j, ufVal);

      Real wfVal = _wf->get(i,j) + coeff*(V.z()-_wf->get(i,j));
      _wf->set(i, j, wfVal);
      flag = 0;
    }
  }
}

Real3 RigidShallowWater::applyFacetForceOnSolid(const Real3& P,const Real3& N,const Real A,const Real3& V,const Real3& VRel)
{

  Real3 Fby(0.0,0.0,0.0), Fdrag(0.0,0.0,0.0), Flift(0.0,0.0,0.0);

  Real h0 = _hf->getSample(Real2(P.x(),P.z()));
  ASSERT(A > 0);
  if(h0 >= P.y())
    Fby.y() = -ShallowWater::_gravity*_rho*A*(h0-P.y())*N.y();
  Real VRelLen = VRel.norm();

  //ASSERT(VRelLen > 0);

  Real Aeff = 0;
  if(h0 < P.y() || N.dot(VRel) < 0 || fabs(VRelLen) < 1e-6)
    Aeff = 0;
  else
    Aeff = (N.dot(VRel)*_omega/VRelLen+(1-_omega))*A;

  Fdrag = -0.5*_CD*Aeff*VRelLen*VRel;

  Real3 NCVRel = N.cross(VRel);
  Real NCVRelL = NCVRel.norm();  

  //ASSERT(NCVRelL > 1e-6);
  if(fabs(NCVRelL) > 1e-6)
    Flift = -0.5*_CL*Aeff*VRelLen*(VRel.cross(NCVRel/NCVRelL));    

  Real3 f = Fby + Fdrag + Flift;
  return f;
}

bool RigidShallowWater::is_sweep_hf(const Real3& P, const Real2& yBB,
                                         const Real3& N, const Real3& VRel) const
{
  int i, j;
  if(!_hf->getId(Real2(P.x(),P.z()), i, j)) {
    return false;
  }

  return (_hf->get(i,j) > yBB.x()) && (_hf->get(i,j) < yBB.y()) &&
         (N.dot(VRel) > 0);
}

Real2 RigidShallowWater::yBoundingBox(const Real3& p1, const Real3& p2, const Real3& p3)
{
  Real PyMin = p1.y(), PyMax = p1.y();

  if(p2.y() > PyMax)
    PyMax = p2.y();
  if(p3.y() > PyMax)
    PyMax = p3.y();

  if(p2.y() < PyMin)
    PyMin = p2.y();
  if(p3.y() < PyMin)
    PyMin = p3.y();
  return Real2(PyMin, PyMax);

}

void RigidShallowWater::subdivision(Real3 v1, Real3 v2, Real3 v3,
                                    Real area, const Real dx,
                                    std::vector<Real3>& newPosData) const
{
  if (area > dx * dx)
  {
    Real3 v12 = (v1 + v2) / 2;
    Real3 v13 = (v1 + v3) / 2;
    Real3 v23 = (v2 + v3) / 2;

    subdivision(v1, v12, v13, 0.25 * area, dx, newPosData);
    subdivision(v2, v23, v12, 0.25 * area, dx, newPosData);
    subdivision(v3, v13, v23, 0.25 * area, dx, newPosData);
    subdivision(v13,v12, v23, 0.25 * area, dx, newPosData);
  }
  else
  {
    newPosData.push_back(v1);
    newPosData.push_back(v2);
    newPosData.push_back(v3);
  }
}

void RigidShallowWater::jitter_position(Real3 & pos, const Real r) const
{
  Real rd1 = random(-r, r);
  Real rd2 = random(r, 3*r);
  Real rd3 = random(-r, r);  
  pos += Real3(rd1, rd2, rd3);
}
