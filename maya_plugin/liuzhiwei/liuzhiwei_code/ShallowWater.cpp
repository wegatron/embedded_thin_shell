#include "ShallowWater.hpp"
#include "ShallowWaterFunctors.hpp"
#include "common/String.hpp"
#include "linearSolver.h"
#ifdef BUILD_GPU_VERSION_ON
#include "solver-gpu/ShallowWaterGpuCuda.hpp"
#endif


USE_PRJ_NAMESPACE;

ShallowWater::ShallowWater(const Real size1,
                           const Real size2,
                           const size_t resolution1,
                           const size_t resolution2,
                           const Real gravity,
                           const Real dt,
                           const Real maxWHeight,
                           const Real maxGHeight,
                           const Real parRadius,
                           const Real Cdeposit)
  : _resolution1(resolution1), _resolution2(resolution2),
    _szCell1(size1/resolution1), _szCell2(size2/resolution2),
    _szInv1(1.0/_szCell1), _szInv2(1.0/_szCell2), _dx(std::min(_szCell1, _szCell2)),
    _gravity(gravity), _dt(dt), _maxWHeight(maxWHeight), _maxGHeight(maxGHeight),

    _epsilon(1e-4*_dx),
    _lambdaE(2 * _dx), _alphaE(0.3),

    _lambdaU(0.1), _lambdaD(0.9),
    _hfTotal(0), _hAvg(0), _water(0), _ground(0), _rho(1000.0f), _frameNum(0),
    _alpMinSplash(0.25), _vMinSplash(2), _lMinSplash(-2), _lambdaY(0.1),
    _Pv(0.01), _dHcap(0.5), 

    _hf(0), _prehf(0),  _gf(0), _uf(0), _wf(0),
    
    _parRadius(parRadius*_dx), _Cdeposit(Cdeposit), _dragCo(0), _foamMaxLife(30)
{
#ifdef BUILD_CPU_VERSION_ON
  INFO("ShallowWater CPU");
#endif  
  init();

#ifdef BUILD_GPU_VERSION_ON
  INFO("ShallowWater GPU");
  // init gpu
  ShallowWaterGpuCuda::initGpu(_resolution1, _resolution2, _szCell1, _szCell2,
                               _gravity, _dt, _epsilon, _dx, _lambdaE, _alphaE);
#endif
}

ShallowWater::~ShallowWater()
{

  if ( _hf ) delete _hf;

  if ( _prehf ) delete _prehf;
  
  if ( _gf ) delete _gf;

  if ( _uf ) delete _uf;

  if ( _wf ) delete _wf;

  // if ( _water ) delete _water; //!!! cann't delete this pointer, because this point to a stack space
  // if ( _ground ) delete _ground; //!!! cann't delete this pointer, because this point to a stack space

#ifdef BUILD_GPU_VERSION_ON  
  // clean gpu
  ShallowWaterGpuCuda::cleanGpu();
#endif
}

void ShallowWater::timestep()
{
  _frameNum++;
   _prehf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_hf));
#ifdef IMPLICIT_ON
  implicit_solve();
#else
  explicit_solve();
#endif

#ifdef TEST_HEIGHT_FIELD_SYMMETRIC_ON
  test_height_field_symmetry();
#endif

  //#define TEST_MASS_CONSERVATION_ON
#ifdef TEST_MASS_CONSERVATION_ON
  test_mass_conservation();
#endif
  //printFrameNum();

// #define TEST_WATERFALL
#ifdef TEST_WATERFALL
  if(_frameNum % 8 == 0)
  {
    const static Real2 center(_resolution1 * _szCell1 * 0.5,
                              _resolution2 * _szCell2 * 0.5);

    const static Real radius = 2.0;
    _hf->apply(ShallowWaterFunctors::Initializer::RealCylinder(
        center, radius, _maxWHeight, _hf));

    // _uf->apply(ShallowWaterFunctors::Initializer::RealCylinder(
    //     center, radius, 2.0, _uf));

    // _wf->apply(ShallowWaterFunctors::Initializer::RealCylinder(
    //     center, radius, 2.0, _wf));
  }
#endif  

}

void ShallowWater::initFromFile(const std::string & hf_filename,
                                const std::string & gf_filename,
                                const std::string & uf_filename,
                                const std::string & wf_filename)
{
  if ( !hf_filename.empty() )
  {
    std::string ext = getLoweredExt(hf_filename);
    if ( ext == "pgm" ) {
      INFO("Initial ground from file: %s", hf_filename.c_str());
      _hf->apply(
          ShallowWaterFunctors::Initializer::
          PGM(hf_filename, _resolution1, _resolution2,
             _hf, _maxWHeight, 0, 0, _resolution1, _resolution2));
    } else {
      ASSERT_MSG(false, "No support found for %s", hf_filename.c_str());
    }
  }

  if ( !gf_filename.empty() )
  {
    std::string ext = getLoweredExt(gf_filename);
    if ( ext == "pgm" ) {
      INFO("Initial ground from file: %s", gf_filename.c_str());
      _gf->apply(
          ShallowWaterFunctors::Initializer::
          PGM(gf_filename, _resolution1, _resolution2,
             _gf, _maxGHeight, 0, 0, _resolution1, _resolution2));
    } else {
      ASSERT_MSG(false, "No support found for %s", gf_filename.c_str());
    }
  }

  // make sure hf >= gf
  _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));
  _hfTotal = _hf->getSummation();
  //_hAvg = _hf->getAverage() - _gf->getAverage();
  if ( !uf_filename.empty() )
  {
    WARN("No support for initialization of velocity filed yet.");
  }

  if ( !wf_filename.empty() )
  {
    WARN("No support for initialization of velocity filed yet.");
  }

  _frameNum = 0;
}

bool ShallowWater::initFromPortableMap(size_t x0, size_t y0,
                                       size_t R1, size_t R2,
                                       PortableMap * water,
                                       PortableMap * ground)
{
  if ( _resolution1 + x0 > R1
    || _resolution2 + y0 > R2 )
  {
    WARN("local size is inconsistent with global window size");
    return false;
  }
  if ( water )
  {
    _water = water;
    _hf->apply(
        ShallowWaterFunctors::Initializer::
        PGM(water, _resolution1, _resolution2,
           _hf, _maxWHeight, x0, y0, R1, R2));
  }
  else WARN("no water PortableMap");
  if ( ground )
  {
    _ground = ground;
    _gf->apply(
        ShallowWaterFunctors::Initializer::
        PGM(ground, _resolution1, _resolution2,
           _gf, _maxGHeight, x0, y0, R1, R2));
  }
  else WARN("no ground PortableMap");

  // make sure hf >= gf
  _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));
  _hfTotal = _hf->getSummation();
  //_hAvg = _hf->getAverage() - _gf->getAverage();
  _frameNum = 0;
  return true;
}

void ShallowWater::setDampParameter(int boundaryIdx, const SIntType dampWidth, const int exp)
{
  set_damping_region(boundaryIdx, dampWidth);

  set_damping_coefficient(boundaryIdx, dampWidth, exp);
}

void ShallowWater::setBoundary(const size_t i, const size_t j)
{
  ASSERT(i < _resolution1 && j < _resolution2);
  _cellType(i, j) = CELL_TYPE::BOUNDARY;
}

void ShallowWater::printParameters() const
{
  INFO("ShallowWater: size = %.2fm x %.2fm", (float)(_resolution1*_szCell1),
                                              (float)(_resolution2*_szCell2));
  INFO("ShallowWater: grid = %ld x %ld", _resolution1, _resolution2);
  INFO("ShallowWater: cell size = %.2fm x %.2fm", (float)(_szCell1),
                                                   (float)(_szCell2));
  INFO("ShallowWater: gravity = %.2f", (float)(_gravity));
  INFO("ShallowWater: dt      = %.2f", (float)(_dt));
}

void ShallowWater::printFrameNum() const
{
  INFO("%d", _frameNum);
}

void ShallowWater::dumpToFile(const std::string & prefix, const size_t X0, const size_t Y0) const
{
  dump_field_to_vtk(prefix+"hf%03d.vtk", _frameNum, X0, Y0, _resolution1, _resolution2, _hf);
#if 0
  dump_field_to_vtk(prefix+"gf%03d.vtk", _frameNum, _resolution1, _resolution2, _gf);
#ifndef IMPLICIT_ON
  dump_field_to_vtk(prefix+"uf%03d.vtk", _frameNum, _resolution1+1, _resolution2, _uf);
  dump_field_to_vtk(prefix+"wf%03d.vtk", _frameNum, _resolution1, _resolution2+1, _wf);
#else
  dump_field_to_vtk(prefix+"uf%03d.vtk", _frameNum, _resolution1, _resolution2, _uf);
  dump_field_to_vtk(prefix+"wf%03d.vtk", _frameNum, _resolution1, _resolution2, _wf);
#endif
#endif
}

void ShallowWater::dumpGroundFieldToMatrix(Matrix& groundMat) const
{
  groundMat.resize(_resolution1, _resolution2);  
  for( size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
      groundMat(i,j) = _gf->get(i,j);
}

void ShallowWater::dumpWaterFieldToMatrix(Matrix& waterMat) const
{
  waterMat.resize(_resolution1,_resolution2);
  for( size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
    {
      waterMat(i,j) = _hf->get(i,j);
      ASSERT(waterMat(i,j) >= 0);
    }
}


void ShallowWater::dumpGroundFieldToVRML(Matrix& groundMat) const
{
   FILE *file_wrl;
   char file_name[20];
   sprintf(file_name,"VRMLFrame%d.wrl",_frameNum);
   
   groundMat.resize(_resolution1, _resolution2);  
   for( size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
      groundMat(i,j) = _hf->get(i,j);
   
   if ((file_wrl = fopen(file_name,"a"))==NULL)//creat dest file(**.obj)
   {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

   	fprintf(file_wrl,"#VRML V2.0 utf8\n");
	fprintf(file_wrl,"Background {\n");
	fprintf(file_wrl,"skyColor 1.0 1.0 1.0\n");
	fprintf(file_wrl,"}\n");

    fprintf(file_wrl,"Shape{\n");
	fprintf(file_wrl,"appearance Appearance {\n");
	fprintf(file_wrl,"material Material {\n");
	fprintf(file_wrl,"diffuseColor  0.0 0.0 0.6\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"geometry ElevationGrid {\n");
	fprintf(file_wrl,"xDimension  %d\n",_resolution1);
	fprintf(file_wrl,"zDimension %d\n",_resolution2);
	fprintf(file_wrl,"xSpacing %.1f\n",100/_resolution1);
	fprintf(file_wrl,"zSpacing %.1f\n",100/_resolution2);
	fprintf(file_wrl,"height [\n");

	for(size_t i = 0;i < _resolution1;i++)
	{
		for(size_t j = 0;j < _resolution2;j++)
		{
		  fprintf(file_wrl,"%.1f ",groundMat(i,j));
		}

		fprintf(file_wrl,"\n");
	}
	fprintf(file_wrl,"	]\n");
	fprintf(file_wrl,"solid FALSE\n");
	fprintf(file_wrl,"creaseAngle	0.0\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"}\n\n");
	  
   	fclose(file_wrl);
   
}
/*

*Summary: Translate dumping water field to VRML2 files.

*Parameters:

*     waterMat : the matrix contains the high degree of water.

*Return : void

*/
void ShallowWater::dumpWaterFieldToVRML(Matrix& waterMat) const
{
   FILE *file_wrl;
   char file_name[512];
   sprintf(file_name,"VRMLFrame%d.wrl",_frameNum);
   float width = (float)100/128;
   
   waterMat.resize(_resolution1, _resolution2);  
   for( size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
      waterMat(i,j) = _hf->get(i,j);
   
   if ((file_wrl = fopen(file_name,"a"))==NULL)//creat dest file(**.obj)
   {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

   	fprintf(file_wrl,"#VRML V2.0 utf8\n");
	fprintf(file_wrl,"Background {\n");
	fprintf(file_wrl,"skyColor 1.0 1.0 1.0\n");
	fprintf(file_wrl,"}\n");

    fprintf(file_wrl,"Shape{\n");
	fprintf(file_wrl,"appearance Appearance {\n");
	fprintf(file_wrl,"material Material {\n");
	fprintf(file_wrl,"diffuseColor  0.0 0.0 0.6\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"geometry ElevationGrid {\n");
	fprintf(file_wrl,"xDimension  %d\n",_resolution1);
	fprintf(file_wrl,"zDimension %d\n",_resolution2);
	fprintf(file_wrl,"xSpacing %.1f\n",width);
	fprintf(file_wrl,"zSpacing %.1f\n",width);
	fprintf(file_wrl,"height [\n");

	for(size_t i = 0;i < _resolution1;i++)
	{
		for(size_t j = 0;j < _resolution2;j++)
		{
		  fprintf(file_wrl,"%.1f ",waterMat(i,j));
		}

		fprintf(file_wrl,"\n");
	}
	fprintf(file_wrl,"	]\n");
	fprintf(file_wrl,"solid FALSE\n");
	fprintf(file_wrl,"creaseAngle	0.0\n");
	fprintf(file_wrl,"	}\n");
	fprintf(file_wrl,"}\n\n");
	  
   	fclose(file_wrl);
   
}



void ShallowWater::dumpVelocityFieldToMatrix(Matrix& hMat, Matrix& vMat) const
{
  hMat.resize(_resolution1+1,_resolution2);
  vMat.resize(_resolution1,_resolution2+1);  

  for( size_t i = 0; i < _resolution1+1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
      hMat(i,j) = _uf->get(i,j);

  for( size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2+1; ++j)
      vMat(i,j) = _wf->get(i,j);
}

void ShallowWater::generateParticles()
{
  for(size_t i = 0; i < _resolution1; ++i)
    for(size_t j = 0; j < _resolution2; ++j)
    {
      if(isBreakingWave(i, j))
        breakingWaveToParticles(i, j);
      
      if(isHorizontalWaterfallFace(i, j))   // right
        hWaterfallToParticles(i, j);

      if(isVerticalWaterfallFace(i, j))  // wrong
        vWaterfallToParticles(i, j);      
    }
  static int flag = 0;
  if(flag < -1)
  {
    printf("-------------------------\n");
    for(size_t i = 0; i < _resolution1; ++i)
    {
      for(size_t j = 0; j < _resolution2; ++j)
      {
        printf("%d ", (int)_cellType(i, j));
      }
      printf("\n");
    }
    
    flag++;
  }
}

void ShallowWater::reduceWaterHeight(const Real totalV, const size_t i, const size_t j)
{
  Real reduceHeight = totalV / (_szCell1 * _szCell2);
  Real h0 = _hf->get(i, j) - reduceHeight;
  Real g0 = _gf->get(i, j);  
  h0 = h0 > g0 ? h0 : g0;
  _hf->set(i, j, h0);

}

Real2 ShallowWater::calGradient(const Field* f, const size_t i, const size_t j) const
{
  Real f0 = f->get(i,j);
  Real fe = f->get(i+1,j);
  Real fw = f->get(i-1,j);

  Real fn = f->get(i,j+1);
  Real fs = f->get(i,j-1);
  
  Real maxabsi = fabs(fe-f0) > fabs(f0-fw) ? (fe-f0):(f0-fw);
  Real maxabsj = fabs(fn-f0) > fabs(f0-fs) ? (fn-f0):(f0-fs);

  Real2 derivF(maxabsi/_szCell1, maxabsj/_szCell2);
  return derivF;
}

bool ShallowWater::isBreakingWave(const size_t i, const size_t j)
{
  Real h0 = _hf->get(i,j);
  Real he = _hf->get(i+1,j);
  Real hw = _hf->get(i-1,j);

  Real hn = _hf->get(i,j+1);
  Real hs = _hf->get(i,j-1);
  
  Real maxabsi = fabs(he-h0) > fabs(h0-hw) ? (he-h0):(h0-hw);
  Real maxabsj = fabs(hn-h0) > fabs(h0-hs) ? (hn-h0):(h0-hs);

  Real2 derivHf(maxabsi/_szCell1, maxabsj/_szCell2);
  
  Real derivHf2 = (he + hw + hn + hs - 4 * h0) / (_szCell1 * _szCell2);

  if((derivHf.norm() > _alpMinSplash * _gravity * _dt / _dx) &&
     ((h0 - _prehf->get(i,j))/_dt > _vMinSplash) &&
     (derivHf2 < _lMinSplash))
  {
    _cellType(i, j) = CELL_TYPE::BREAKING_WAVE;
    return true;
  }
  else
    return false;
}

void ShallowWater::breakingWaveToParticles(const size_t i, const size_t j)
{
  ///< breaking wave const coefficients
  static const Real Proportion = 1.0;
  static const Real HAvg = _hf->getAverage() - _gf->getAverage();
  Real sprayDragCo = 0.1;
    
  Real2 derivHf = calGradient(_hf, i, j);
  Real dHfNorm = derivHf.norm();
  Real totalV = Proportion * dHfNorm;

  Real h0 = _hf->get(i, j);
  Real H0 = h0 - _gf->get(i, j);
 
  Real2 centerPos = _hf->getPosition(i, j);

  //us = (1 + pv g(H(x) − Hi ))ul 
  Real vx = (1+_Pv*_gravity*(H0-HAvg)) * _uf->get(i, j);

  Real vz = (1+_Pv*_gravity*(H0-HAvg)) * _wf->get(i, j);

  Real vy = _lambdaY * (h0 - _prehf->get(i,j)) / _dt;

  ///< clampV is not used, but effect is better
  // ASSERT(dHfNorm > 0);
  // Real2 clampV(0, 0);
  // if(dHfNorm > 1e-6)
  //   Real2 clampV = (-sqrt(_gravity*std::min<Real>(H0, HAvg))/dHfNorm)*derivHf;
  // vx = vx < clampV.x() ? vx:clampV.x();
  // vz = vz < clampV.y() ? vz:clampV.y();  

  Real vol = 0;
  while(vol < totalV)
  {
    Real rd = random(-_dx*0.5, _dx*0.5);
    Real3 pos(centerPos.x()+rd, h0+rd, centerPos.y()+rd);
    Real r = random(0.7 * _parRadius, _parRadius);
    vol += Particle::calVolume(r);
    Real3 v(vx, vy, vz);
    Particle par(pos, v, r, sprayDragCo);
    _spray.push_back(par);
  }
  //reduceWaterHeight(totalV, i, j);
}

bool ShallowWater::isHorizontalWaterfallFace(const size_t i, const size_t j)
{
  Real g0 = _gf->get(i, j);
  Real gE = _gf->get(i+1, j);
  Real h0 = _hf->get(i, j);
  Real hE = _hf->get(i+1, j);
  bool case1 = (g0-gE)/_szCell1 > _dHcap && g0 > hE;
  bool case2 = (gE-g0)/_szCell1 > _dHcap && gE > h0;  
  if(case1 || case2)
  {
    if(case1)
      _cellType(i, j) = CELL_TYPE::HORIZON_WATERFALL1;
    else
      _cellType(i, j) = CELL_TYPE::HORIZON_WATERFALL2;
    return true;
  }
  else
    return false;

}

bool ShallowWater::isVerticalWaterfallFace(const size_t i, const size_t j)
{

  Real g0 = _gf->get(i, j);
  Real gN = _gf->get(i, j+1);
  Real h0 = _hf->get(i, j);
  Real hN = _hf->get(i, j+1);
  bool case1 = (g0-gN)/_szCell2 > _dHcap && g0 > hN;
  bool case2 = (gN-g0)/_szCell2 > _dHcap && gN > h0;  
  if(case1 || case2)
  {
    if(case1)
      _cellType(i, j) = CELL_TYPE::VERTICAL_WATERFALL1;
    else
      _cellType(i, j) = CELL_TYPE::VERTICAL_WATERFALL2;
    return true;
  }
  else
    return false;
}

void ShallowWater::generateWaterfallPars(const Real3 Vwf, const Real3 Pmin, const Real3 Pmax)
{
  Real r = 3 * _parRadius;
  Real splashDragCo = 0.2;
  
  Real3 bbWf = Pmax - Pmin;
  Real totalV = bbWf.x()*bbWf.y()*bbWf.z();
  size_t totalNum = (size_t)totalV / Particle::calVolume(r);

  //INFO("Waterfall particle number:%d", totalNum);
  
  Real3 centerPos = (Pmax+Pmin)/2.0;

  Real vol = 0;
  Real parV = Particle::calVolume(r);
  
  for(size_t n = 0; n < totalNum; ++n)
    //while(totalV - vol > parV)
  {
    Real rdX = random(-bbWf.x()/2.0, bbWf.x()/2.0);
    Real rdY = random(-bbWf.y()/2.0, bbWf.y()/2.0);
    Real rdZ = random(-bbWf.z()/2.0, bbWf.z()/2.0);    
    Real3 loc = centerPos + Real3(rdX, rdY, rdZ);

    r = random(3 * _parRadius, 4 * _parRadius);
    // vol += Particle::calVolume(r);

    //INFO("vol: %f", vol);
    Particle par(loc, Vwf, r, splashDragCo);
    _splash.push_back(par);
    
  }
}

// void ShallowWater::hWaterfallToParticles(const size_t i, const size_t j)
// {
//   Real h0 = _hf->get(i, j);
//   Real hE = _hf->get(i+1, j);
  
//   Real g0 = _gf->get(i, j);
//   Real gE = _gf->get(i+1, j);
  
//   Real uE = _uf->get(i+1, j);
  
//   Real wN = _wf->get(i, j+1);
//   Real wS = _wf->get(i, j);

//   Real wfEN = _wf->get(i+1, j+1);

//   Real wfES = _wf->get(i+1, j);

//   Real3 Vwf, Pmin, Pmax;

//   if(hE < g0)
//   {
//     Vwf = Real3(uE, 0, (wN+wS)/2);
//     Pmin = Real3(i*_szCell1, g0, j*_szCell2);
//     Pmax = Real3(i*_szCell1 + uE*_dt, h0, (j+1)*_szCell2);
//   }
//   else
//   {
//     Vwf = Real3(uE, 0, (wfEN+wfES)/2);
//     Pmin = Real3((i+1)*_szCell1+uE*_dt, gE, j*_szCell2);
//     Pmax = Real3((i+1)*_szCell1, hE, (j+1)*_szCell2);
//   }
//   generateWaterfallPars(Vwf, Pmin, Pmax);
  
// }

void ShallowWater::hWaterfallToParticles(const size_t i, const size_t j)
{
  Real h0 = _hf->get(i, j);
  Real hE = _hf->get(i+1, j);
  
  Real g0 = _gf->get(i, j);
  Real gE = _gf->get(i+1, j);
  
  Real uE = _uf->get(i+1, j);
  
  Real wN = _wf->get(i, j+1);
  Real wS = _wf->get(i, j);

  Real wfEN = _wf->get(i+1, j+1);

  Real wfES = _wf->get(i+1, j);

  Real3 Vwf, Pmin, Pmax;

  if(hE < g0)
  {
    Vwf = Real3(uE, 0, (wN+wS)/2);
    Pmin = Real3(i*_szCell1, g0, j*_szCell2);
    Pmax = Real3(i*_szCell1 + uE*_dt, h0, (j+1)*_szCell2);
  }
  else
  {
    Vwf = Real3(uE, 0, (wfEN+wfES)/2);
    Pmin = Real3((i+1)*_szCell1+uE*_dt, gE, j*_szCell2);
    Pmax = Real3((i+1)*_szCell1, hE, (j+1)*_szCell2);
  }
  generateWaterfallPars(Vwf, Pmin, Pmax);
  
}


void ShallowWater::vWaterfallToParticles(const size_t i, const size_t j)
{
  Real h0 = _hf->get(i, j);
  Real hN = _hf->get(i, j+1);
  
  Real g0 = _gf->get(i, j);
  Real gN = _gf->get(i, j+1);
  
  Real wN = _wf->get(i, j+1);

  Real uE = _uf->get(i+1, j);
  Real uW = _uf->get(i, j);

  Real ufEN = _uf->get(i+1, j+1);
  Real ufWN = _uf->get(i, j+1);

  Real3 Vwf, Pmin, Pmax;

  if(hN < g0)
  {
    Vwf = Real3((uE+uW)/2, 0, wN);
    Pmin = Real3(i*_szCell1, g0, j*_szCell2);
    Pmax = Real3((i+1)*_szCell1, h0, j*_szCell2 + wN*_dt);
  }
  else
  {
    Vwf = Real3((ufEN+ufWN)/2, 0, wN);
    Pmin = Real3(i*_szCell1, gN, (j+1)*_szCell2+wN*_dt);
    Pmax = Real3((i+1)*_szCell1, hN, (j+1)*_szCell2);
  }

  generateWaterfallPars(Vwf, Pmin, Pmax);    
  
}

void ShallowWater::particleToHeightField(Particle& fallPar)
{
  Real3 parPos = fallPar.getPosition();
  Real2 pos(parPos.x(), parPos.z());
  Real dx2 = _szCell1 * _szCell2;
  int i, j;
  //srand(time(NULL));
  if(_hf->getId(pos, i, j))
  {
    Real3 parV = fallPar.getVelocity();

    if(parV.norm() < 100 && rand() % 100 < 50) /// probility = 0.5
      turnParticleToFoam(fallPar, pos);
    ///< Add particle to height field
    else
    {
      // Real parVol = fallPar.getVolume();
      // //Real hfVal = _hf->get(i, j) + parVol/(dx2);
      // Real hfVal = _hf->get(i, j);
      // _hf->set(i, j, hfVal);

      // Real h0 = _hf->get(i, j);

      // Real ufVal = (_uf->get(i, j)*h0*dx2+parV.x()*parVol)/(h0*dx2+parVol);
      // _uf->set(i, j, ufVal);
    
      // Real wfVal = (_wf->get(i, j)*h0*dx2+parV.z()*parVol)/(h0*dx2+parVol);
      // _wf->set(i, j, wfVal);
    }
  }
}

void ShallowWater::turnParticleToFoam(Particle& fallPar, Real2 pos)
{

  Real3 VF(_uf->getSample(pos), 0.0f, _wf->getSample(pos));
  VF.y() = Real2(VF.x(),VF.z()).dot(_hf->getDSample(pos));
  Real hfVal = _hf->getSample(pos);

  //srand(time(NULL));
  Particle foam(Real3(pos.x(), hfVal, pos.y()), VF, _parRadius, rand() % _foamMaxLife);

  //fallPar.turnToFoam(VF, hfVal, rand() % _foamMaxLife);
  
  _foams.push_back(foam);
  
}


BytesPackage ShallowWater::getRegionPackage(const size_t x0, const size_t y0,
                                            const size_t x1, const size_t y1,
                                            const size_t X0, const size_t Y0,
                                            const size_t R1, const size_t R2)
                                            const
{
  // make sure local region is within global region
  ASSERT_MSG((X0+_resolution1<=R1)&&(Y0+_resolution2<=R2),
      "X0 _resolution R1 Y0 _resolution2 R2 = %ld %ld %ld %ld %ld %ld",
      X0, _resolution1, R1, Y0, _resolution2, R2);
  //  make sure the boundary region is valid
  ASSERT(x0<x1&&y0<y1);
  ASSERT(X0<=x0&&Y0<=y0);
  ASSERT((x0<=X0+_resolution1)&&(y0<=Y0+_resolution2));
  // make sure data are valid
  ASSERT(_hf&&_gf&&_uf&&_wf);

  size_t dlx = 1;
  size_t dly = 1;
#ifdef IMPLICIT_ON
  dlx = 0;
  dly = 0;
#endif
  const size_t lx0 = x0 - X0;
  const size_t ly0 = y0 - Y0;
  const size_t lx1 = x1 - X0;
  const size_t ly1 = y1 - Y0;
  BytesPackage bp;
  std::string message = "Hello";
  bp.addString(message);

  // Assumption: _gf is const.
  for ( size_t i=lx0; i < lx1; i++ )
    for ( size_t j=ly0; j < ly1; j++ )
    {
      bp.add(_hf->get(i, j));
    }
  for ( size_t i=lx0; i < lx1+dlx; i++ )
    for ( size_t j=ly0; j < ly1; j++ )
    {
      bp.add(_uf->get(i, j));
    }
  for ( size_t i=lx0; i < lx1; i++ )
    for ( size_t j=ly0; j < ly1+dly; j++ )
    {
      bp.add(_wf->get(i, j));
    }
  return bp;
}

bool ShallowWater::setRegionPackage(BytesPackage & bp,
                                    const size_t x0, const size_t y0,
                                    const size_t x1, const size_t y1,
                                    const size_t X0, const size_t Y0,
                                    const size_t R1, const size_t R2)
{
  // make sure local region is within global region
  ASSERT_MSG((X0+_resolution1<=R1)&&(Y0+_resolution2<=R2),
      "X0 _resolution R1 Y0 _resolution2 R2 = %ld %ld %ld %ld %ld %ld",
      X0, _resolution1, R1, Y0, _resolution2, R2);
  //  make sure the boundary region is valid
  ASSERT(x0<x1&&y0<y1);
  ASSERT(X0<=x0&&Y0<=y0);
  ASSERT((x0<=X0+_resolution1)&&(y0<=Y0+_resolution2));
  // make sure data are valid
  ASSERT(_hf&&_gf&&_uf&&_wf);

  size_t dlx = 1;
  size_t dly = 1;
#ifdef IMPLICIT_ON
  dlx = 0;
  dly = 0;
#endif
  const size_t lx0 = x0 - X0;
  const size_t ly0 = y0 - Y0;
  const size_t lx1 = x1 - X0;
  const size_t ly1 = y1 - Y0;
  std::string message;
  bp.getString(message);
  if ( message != "Hello" )
    return false;
  for ( size_t i=lx0; i < lx1; i++ )
    for ( size_t j=ly0; j < ly1; j++ )
    {
      bp.get(_hf->get(i, j));
    }
  for ( size_t i=lx0; i < lx1+dlx; i++ )
    for ( size_t j=ly0; j < ly1; j++ )
    {
      bp.get(_uf->get(i, j));
    }
  for ( size_t i=lx0; i < lx1; i++ )
    for ( size_t j=ly0; j < ly1+dly; j++ )
    {
      bp.get(_wf->get(i, j));
    }
  return true;
}

void ShallowWater::init()
{
  // allocate fileds
  {
    _hf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::CENTER);
    _prehf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::CENTER);
    
    _gf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::CENTER);
#ifdef IMPLICIT_ON
    _uf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::CENTER);
    _wf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::CENTER);
#else
    _uf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::BORDER_HORIZONTAL);
    _wf = new Field(_resolution1, _resolution2, Real2(_szCell1, _szCell2), Field::BORDER_VERTICAL);
#endif
    ASSERT(_hf && _gf && _uf && _wf);
  }

  ///< allocate matrix
  {
    _phi.resize(_resolution1, _resolution2);
    _psi.resize(_resolution1, _resolution2);
    _sigma.resize(_resolution1, _resolution2);
    _gamma.resize(_resolution1, _resolution2);
    _cellType.resize(_resolution1, _resolution2);
  }

  ///< init fields
  {
    // init water depth (>= 0)
    _hf->apply(ShallowWaterFunctors::Initializer::Zeros<Real>());

    // init ground height
    _gf->apply(ShallowWaterFunctors::Initializer::Zeros<Real>());

    // make sure hf >= gf
    _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));

    // init velocity u
    _uf->apply(ShallowWaterFunctors::Initializer::Zeros<Real>());
    // init velocity w
    _wf->apply(ShallowWaterFunctors::Initializer::Zeros<Real>());

  }

  ///< init matrix
  {
    _phi.setZero();
    _psi.setZero();    
    _sigma.setZero();    
    _gamma.setZero();
    _cellType.setZero();
  }
  _hfTotal = _hf->getSummation();
  //_hAvg = _hf->getAverage() - _gf->getAverage();

  std::srand(std::time(NULL));
  
  INFO("Shallow water initialized!");
}

void ShallowWater::explicit_solve()
{
#ifdef IMPLICIT_ON
  WARN("Using explicit method denied when implicit is on.");
  return;
#endif

  // build cpu version
#ifdef BUILD_CPU_VERSION_ON
  Field * uf = new Field(*_uf);
  Field * wf = new Field(*_wf);
  ASSERT(uf && wf);
  uf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_uf));
  wf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_wf));

  // velocity advection for u
  // input: uf, wf, _dt
  // output: _uf
  _uf->apply(ShallowWaterFunctors::Advector::
#ifdef ADVECT_SEMI_LAGRANGIAN
    SemiLagrangian<Real, Real>(uf, wf, _dt, uf)
#else
    MacCormack<Real, Real>(uf, wf, _dt, uf)
#endif
  );
  // velocity advection for w
  // input: uf, wf, _dt
  // output: _wf
  _wf->apply(ShallowWaterFunctors::Advector::
#ifdef ADVECT_SEMI_LAGRANGIAN
    SemiLagrangian<Real, Real>(uf, wf, _dt, wf)
#else
    MacCormack<Real, Real>(uf, wf, _dt, wf)
#endif
  );

  // delete temporay variables
  delete uf;
  delete wf;

  // stability enhancements
  // input: _uf, _wf, cfl_velocity
  // output: _uf, _wf
  _uf->apply(ShallowWaterFunctors::Clamper::
    MaxAbsSuppress<Real>(_szCell1/2/_dt, _uf)
  );
  _wf->apply(ShallowWaterFunctors::Clamper::
    MaxAbsSuppress<Real>(_szCell2/2/_dt, _wf)
  );

  // height integration
  // input: _uf, _wf, _szCell, _gravity, _dt, _hf
  // output: _hf
  _hf->apply(ShallowWaterFunctors::Integrator::
             UpwindSWE<Real, Real>(_uf, _wf, _gf, Real2(_szCell1, _szCell2),
                                   _gravity, _dt, _hf, _cellType));

  // stability enhancements to make sure _hf >= _gf
  // input: _hf, _gf
  // output: _hf
  _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));

  // do velocity integration
  // input: _hf, _szCell, _gravity, _dt, _uf, _wf
  // output: _uf, _wf
  _uf->apply(ShallowWaterFunctors::Integrator::
     VelocitySWE<Real, Real>(_gf,_hf, Real2(_szCell1, _szCell2),
                             _gravity, _dt, _uf, _cellType));

  _wf->apply(ShallowWaterFunctors::Integrator::
     VelocitySWE<Real, Real>(_gf,_hf, Real2(_szCell1, _szCell2),
                             _gravity, _dt, _wf, _cellType));

  // stability enhancements
  // input: _uf, _wf, cfl_velocity
  // output: _uf, _wf
  _uf->apply(ShallowWaterFunctors::Clamper::
    MaxAbsSuppress<Real>(_szCell1/_dt, _uf)
  );
  _wf->apply(ShallowWaterFunctors::Clamper::
    MaxAbsSuppress<Real>(_szCell2/_dt, _wf)
  );

  // apply relfective boundary condition
  // input: _hf, _gf, _uf, _wf
  // output: _uf, _wf
  _uf->apply(ShallowWaterFunctors::Boundary::
    Reflective<Real, Real>(_hf, _gf, _epsilon, _uf)
  );

  _wf->apply(ShallowWaterFunctors::Boundary::
    Reflective<Real, Real>(_hf, _gf, _epsilon, _wf)
  );

  // Real hRest = _hf->getAverage() - _gf->getAverage();
  // // apply damping region condition for water height
  // _hf->apply(ShallowWaterFunctors::Damper::
  //            PMLHeight<Real, Real>(_gf, _hf, _phi, _sigma, _dt, hRest));
  // hRest = _hf->getAverage() - _gf->getAverage();
  // _hf->apply(ShallowWaterFunctors::Damper::
  //            PMLHeight<Real, Real>(_gf, _hf, _psi, _gamma, _dt, hRest));
  // // stability enhancements to make sure _hf >= _gf
  // _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));
  
  // // apply damping region condition for velocity
  // _uf->apply(ShallowWaterFunctors::Damper::
  //            PMLVelocity<Real, Real>(_gf, _uf, _sigma, _dt));
  // _wf->apply(ShallowWaterFunctors::Damper::
  //            PMLVelocity<Real, Real>(_gf, _wf, _gamma, _dt));
  // // stability enhancements
  // _uf->apply(ShallowWaterFunctors::Clamper::
  //            MaxAbsSuppress<Real>(_szCell1/2/_dt, _uf));
  // _wf->apply(ShallowWaterFunctors::Clamper::
  //            MaxAbsSuppress<Real>(_szCell2/2/_dt, _wf));
  // // modify damping coefficient along x,z            
  // _phi->apply(ShallowWaterFunctors::Damper::
  //             PMLPhi<Real, Real>(_gf, _wf, _sigma, _phi, _dt, _lambdaD, _lambdaU,
  //                                Real2(_szCell1, _szCell2)));
  // _psi->apply(ShallowWaterFunctors::Damper::
  //             PMLPhi<Real, Real>(_gf, _uf, _gamma, _psi, _dt, _lambdaD, _lambdaU,
  //                                Real2(_szCell1, _szCell2)));

  // apply overshooting reduction
  _hf->apply(ShallowWaterFunctors::OvershootingReducer::
             ReduceHeight<Real, Real>(_hf, _uf, _lambdaE, _alphaE));
  // apply overshooting reduction
  _hf->apply(ShallowWaterFunctors::OvershootingReducer::
             ReduceHeight<Real, Real>(_hf, _wf, _lambdaE, _alphaE));
  //stability enhancements to make sure _hf >= _gf
  _hf->apply(ShallowWaterFunctors::Clamper::Max<Real>(_hf, _gf));
#endif
  
  // build Gpu version
#ifdef BUILD_GPU_VERSION_ON
  ShallowWaterGpuCuda::timestepGpu(_hf->getRawData(),
                                   _gf->getRawData(),
                                   _uf->getRawData(),
                                   _wf->getRawData());
#endif
}


void ShallowWater::implicit_solve()
{
#ifndef IMPLICIT_ON
  WARN("Using implicit method denied.");
  return;
#endif

  struct ToLinear : public Functor2<size_t, size_t> {
    size_t _n;
    ToLinear(size_t n) : _n(n) {}
    inline size_t operator()(const size_t & i, const size_t & j) const
    {
      return i * _n + j;
    }
  } toLinear(_resolution2);

  Field * uf = new Field(*_uf);
  Field * wf = new Field(*_wf);
  Field * hf = new Field(*_hf);
  ASSERT(uf && wf && hf);
  uf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_uf));
  wf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_wf));
  hf->apply(ShallowWaterFunctors::Initializer::Identity<Real>(_hf));

  // velocity advection for u
  // input: uf, wf, _dt
  // output: _uf
  _uf->apply(ShallowWaterFunctors::Advector::
#ifdef ADVECT_SEMI_LAGRANGIAN
    SemiLagrangian<Real, Real>(uf, wf, _dt, uf)
#else
    MacCormack<Real, Real>(uf, wf, _dt, uf)
#endif
  );

  // velocity advection for w
  // input: uf, wf, _dt
  // output: _wf
  _wf->apply(ShallowWaterFunctors::Advector::
#ifdef ADVECT_SEMI_LAGRANGIAN
    SemiLagrangian<Real, Real>(uf, wf, _dt, wf)
#else
    MacCormack<Real, Real>(uf, wf, _dt, wf)
#endif
  );

  // height advection for h
  // input: uf, wf, _dt, hf
  // output: _hf
  _hf->apply(ShallowWaterFunctors::Advector::
#ifdef ADVECT_SEMI_LAGRANGIAN
    SemiLagrangian<Real, Real>(uf, wf, _dt, hf)
#else
    MacCormack<Real, Real>(uf, wf, _dt, hf)
#endif
  );

  const Real ttg = _dt * _dt * _gravity;
  const Real dx2Inv = _szInv1 * _szInv1;
  const Real dy2Inv = _szInv2 * _szInv2;

  sparseMatrix<Real> theMatrix(_resolution1*_resolution2, _resolution1*_resolution2);
  PCGSolver<Real>::vec result(_resolution1*_resolution2);
  PCGSolver<Real>::vec rhs(_resolution1*_resolution2);

  ASSERT(_uf->getStorageType()==_hf->getStorageType());
  for ( size_t i=0; i < _resolution1; i++ )
    for ( size_t j=0; j < _resolution2; j++ )
    {
      const Real2 pos = _hf->getPosition(i, j);
      const Real2 pos_e = ((i<_resolution1-1)?_hf->getPosition(i+1, j):_hf->getPosition(i,j));
      const Real2 pos_w = ((i>0)?_hf->getPosition(i-1, j):_hf->getPosition(i,j));
      const Real2 pos_n = ((j<_resolution2-1)?_hf->getPosition(i, j+1):_hf->getPosition(i,j));
      const Real2 pos_s = ((j>0)?_hf->getPosition(i, j-1):_hf->getPosition(i,j));

      const Real b = _gf->getSample(pos); // ground height
      const Real h = hf->getSample(pos);  // old water surface height
      const Real d = h - b;               // old water depth
      const Real b_e = _gf->getSample(pos_e);
      const Real b_w = _gf->getSample(pos_w);
      const Real b_n = _gf->getSample(pos_n);
      const Real b_s = _gf->getSample(pos_s);

      const Real u = _uf->getSample(pos);     // new velocity
      const Real w = _wf->getSample(pos);     // new velocity
      const Real u_e = _uf->getSample(pos_e); // new velocity
      const Real u_w = _uf->getSample(pos_w); // new velocity
      const Real w_n = _wf->getSample(pos_n); // new velocity
      const Real w_s = _wf->getSample(pos_s); // new velocity

      const Real h_tilda = _hf->getSample(pos); // new water surface height
      rhs(toLinear(i, j)) = h_tilda + _dt*(0.5*_szInv1*u*(b_e-b_w)+
                                           0.5*_szInv2*w*(b_n-b_s))
                                    - _dt*d*(0.5*_szInv1*(u_e-u_w)+
                                             0.5*_szInv2*(w_n-w_s));

      theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+2*ttg*d*(dx2Inv+dy2Inv));
      if ( i < _resolution1-1)
      theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*(0.25*dx2Inv)*(b_e-b_w-4*d));
      if ( j < _resolution2-1 )
      theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*(0.25*dy2Inv)*(b_n-b_s-4*d));
      if ( i > 0 )
      theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*(0.25*dx2Inv)*(b_w-b_e-4*d));
      if ( j > 0 )
      theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*(0.25*dy2Inv)*(b_s-b_n-4*d));

      if ( i == _resolution1-1 )
      {
        if ( j == _resolution2-1 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*dx2Inv*(b_w-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*dy2Inv*(b_s-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_w)+dy2Inv*(b-b_s)));
        }
        else if ( j == 0 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*dx2Inv*(b_w-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*dy2Inv*(b_n-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_w)+dy2Inv*(b-b_n)));
        }
        else
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*dx2Inv*(b_w-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*(0.25*dy2Inv)*(b_n-b_s-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*(0.25*dy2Inv)*(b_s-b_n-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_w)+dy2Inv*2*d));
        }
      }
      else if ( i == 0 )
      {
        if ( j == _resolution2-1 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*dx2Inv*(b_e-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*dy2Inv*(b_s-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_e)+dy2Inv*(b-b_s)));
        }
        else if ( j == 0 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*dx2Inv*(b_e-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*dy2Inv*(b_n-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_e)+dy2Inv*(b-b_n)));
        }
        else
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*dx2Inv*(b_e-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*(0.25*dy2Inv)*(b_n-b_s-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*(0.25*dy2Inv)*(b_s-b_n-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dx2Inv*(b-b_e)+dy2Inv*2*d));
        }
      }
      else
      {
        if ( j == _resolution2-1 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i,j-1), ttg*dy2Inv*(b_s-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*(0.25*dx2Inv)*(b_e-b_w-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*(0.25*dx2Inv)*(b_w-b_e-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dy2Inv*(b-b_s)+dx2Inv*2*d));
        }
        else if ( j == 0 )
        {
          theMatrix.setElement(toLinear(i,j), toLinear(i,j+1), ttg*dy2Inv*(b_n-b));
          theMatrix.setElement(toLinear(i,j), toLinear(i+1,j), ttg*(0.25*dx2Inv)*(b_e-b_w-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i-1,j), ttg*(0.25*dx2Inv)*(b_w-b_e-4*d));
          theMatrix.setElement(toLinear(i,j), toLinear(i,j), 1+ttg*(dy2Inv*(b-b_n)+dx2Inv*2*d));
        }
      }
    }

  // delete temporay variables
  delete uf;
  delete wf;
  delete hf;

  ASSERT(theMatrix.isDiagonalDominant());
  ASSERT(theMatrix.isSymmetric(0.5));

  PCGSolver<Real> mysolver;
  mysolver.setMatrix(theMatrix);
  ASSERT_MSG(mysolver.solve(rhs, result), "Fail to converge");

  // update height field
  for ( size_t i=0; i < _resolution1; i++ )
    for ( size_t j=0; j < _resolution2; j++ )
    {
      _hf->get(i, j) = result(toLinear(i,j));
    }

  // update velocity fields
  ASSERT(_uf->getStorageType()==_hf->getStorageType());
  for ( size_t i=0; i < _resolution1; i++ )
    for ( size_t j=0; j < _resolution2; j++ )
    {
      const Real h_e = _hf->getSample(_hf->getPosition(i+1, j));
      const Real h_w = _hf->getSample(_hf->getPosition(i-1, j));
      const Real h_n = _hf->getSample(_hf->getPosition(i, j+1));
      const Real h_s = _hf->getSample(_hf->getPosition(i, j-1));
      _uf->get(i, j) = _uf->get(i, j) - 0.5*_szInv1*_dt*_gravity*(h_e-h_w);
      _wf->get(i, j) = _wf->get(i, j) - 0.5*_szInv2*_dt*_gravity*(h_n-h_s);
    }

  // boundary condition
  for ( size_t i=0; i < _resolution1; i++ )
  {
    _wf->get(i, 0) = 0;
    _wf->get(i, _resolution2-1) = 0;
  }
  for ( size_t j=0; j < _resolution2; j++ )
  {
    _uf->get(0, j) = 0;
    _uf->get(_resolution1-1, j) = 0;
  }
}

void ShallowWater::test_height_field_symmetry()
{
  if ( !_hf->isSymmetric() )
  {
    WARN("%d: hf not symmetric", _frameNum);
  }
#if 0
  else
  {
    INFO("%d: hf symmetric", _frameNum);
  }
#endif
}

void ShallowWater::test_mass_conservation()
{
  Real h = _hf->getSummation();
  if ( !isZero(h-_hfTotal) )
  {
#ifdef DOUBLE_PRECISION
    WARN("Frame %d: Mass changed %.10lf -> %.10lf, %.10lf", _frameNum, _hfTotal, h, h-_hfTotal);
#else
    WARN("Frame %d: Mass changed %.6f -> %.6f, %.6f", _frameNum, _hfTotal, h, h-_hfTotal);
#endif
    _hfTotal = h;
  }
#if 0
  else
  {
    INFO("Frame %d: Mass conserved.", _frameNum);
  }
#endif
}

void ShallowWater::dump_field_to_vtk(const std::string & format,
                                     const int index,
                                     const size_t X0,
                                     const size_t Y0,
                                     const size_t resolution1,
                                     const size_t resolution2,
                                     const ShallowWater::Field * tf)
{
  if ( !tf ) {
    WARN("Null field!");
    return;
  }
  char buff[BUFSIZ];
  sprintf(buff, format.c_str(), index);
  FILE * fp = fopen(buff, "w");
  ASSERT(fp);

  fprintf(fp, "# vtk DataFile Version 1.0\n");
  fprintf(fp, "xx field\n");
  fprintf(fp, "ASCII\n");
  fprintf(fp, "\nDATASET POLYDATA\n");
#ifdef DOUBLE_PRECISION
  fprintf(fp, "POINTS %ld double\n", resolution1 * resolution2);
#else
  fprintf(fp, "POINTS %ld float\n", resolution1 * resolution2);
#endif
  for ( size_t x1=0; x1 < resolution1; x1++ )
    for ( size_t x2=0; x2 < resolution2; x2++ )
    {
      Real2 p = tf->getPosition(x1, x2);
      Real2 q = tf->getPosition(x1+X0, x2+Y0);
#ifdef DOUBLE_PRECISION
      fprintf(fp, "%.10lf %.10lf %.10lf\n", q(0), q(1), tf->getSample(p));
#else
      fprintf(fp, "%f %f %f\n", q(0), q(1), tf->getSample(p));
#endif
    }
  fprintf(fp, "\nPOLYGONS %ld %ld\n", (resolution1-1)*(resolution2-1), 5*(resolution1-1)*(resolution2-1));
  for ( size_t x1=0; x1 < resolution1-1; x1++ )
    for ( size_t x2=0; x2 < resolution2-1; x2++ )
    {
#define F(a,b) ((a)*(resolution2)+(b))
      fprintf(fp, "4 %ld %ld %ld %ld\n", F(x1,x2), F(x1+1,x2), F(x1+1,x2+1), F(x1,x2+1));
#undef F
    }
  fprintf(fp, "\nPOINT_DATA %ld\n", resolution1 * resolution2);
#ifdef DOUBLE_PRECISION
  fprintf(fp, "SCALARS height double\n");
#else
  fprintf(fp, "SCALARS height float\n");
#endif
  fprintf(fp, "LOOKUP_TABLE custom_table\n");
  for ( size_t x1=0; x1 < resolution1; x1++ )
    for ( size_t x2=0; x2 < resolution2; x2++ )
    {
      Real2 p = tf->getPosition(x1, x2);
#ifdef DOUBLE_PRECISION
      fprintf(fp, "%.10lf\n", tf->getSample(p));
#else
      fprintf(fp, "%f\n", tf->getSample(p));
#endif
    }
  fprintf(fp, "\n");
  fclose(fp);
}

/** Set Damping Region.
 * @param boundaryIdx the index of damping boundary
 *                     (left 0,top 1,right 2, bottom 3)
 * @param dampWidth
 */
void ShallowWater::set_damping_region(int boundaryIdx, const size_t dampWidth)
{
  boundaryIdx %= 4;
  switch(boundaryIdx) {
    case 0: // left
      for (size_t i= 0; i < dampWidth; i++ )
        for (size_t j= 0; j < _resolution2; j++ )
          _cellType(i, j) = CELL_TYPE::DAMPING_REGION;
      break;
    case 1: // top
      for (size_t i= 0; i < _resolution1; i++ )
        for (size_t j= _resolution2-dampWidth; j < _resolution2; j++ )
          _cellType(i, j) = CELL_TYPE::DAMPING_REGION;
      break;
    case 2: // right
      for (size_t i= _resolution1-dampWidth; i < _resolution1; i++ )
        for (size_t j= 0; j < _resolution2; j++ )
          _cellType(i, j) = CELL_TYPE::DAMPING_REGION;
      break;
    case 3: // bottom
      for (size_t i= 0; i < _resolution1; i++ )
        for (size_t j= 0; j < dampWidth; j++ )
          _cellType(i, j) = CELL_TYPE::DAMPING_REGION;
      break;
  }
}

/** Set Damping Region coefficient.
 * @param boundaryIdx the index of damping boundary
 *                     (left 0,top 1,right 2, bottom 3)
 * @param dampWidth
 * @param exp quadratically or cubically increase from inner edge
 */
void ShallowWater::set_damping_coefficient(int boundaryIdx, const size_t dampWidth,
                                           const int exp)
{
  boundaryIdx %= 4;
  switch(boundaryIdx) {
    case 0: // left
      for(size_t i = 0; i < dampWidth; i++) {
        Real dis = (Real)(dampWidth - i) / dampWidth;
        Real co = std::pow(dis, exp);
        for (size_t j = 0; j < _resolution2; j++ )
          _sigma(i, j) = co;
      }
      break;
    case 1: // top
      for(size_t j= _resolution2-dampWidth; j < _resolution2; j++) {
        Real dis = (Real)(j-(_resolution2-dampWidth)) / dampWidth;
        Real co = std::pow(dis, exp);
        for (size_t i = 0; i < _resolution1; i++ )
          _gamma(i, j) = co;
      }
      break;
    case 2: // right
      for(size_t i= _resolution1-dampWidth; i < _resolution1; i++) {
        Real dis = (Real)(i-(_resolution1-dampWidth)) / dampWidth;
        Real co = std::pow(dis, exp);
        for (size_t j = 0; j < _resolution2; j++ )
          _sigma(i, j) = co;
      }
      break;
    case 3: // bottom
      for( size_t j = 0; j < dampWidth; j++) {
        Real dis = (Real)(dampWidth-j) / dampWidth;
        Real co = std::pow(dis, exp);
        for (size_t i = 0; i < _resolution1; i++)
          _gamma(i, j) = co;        
      }
      break;
  }
}
