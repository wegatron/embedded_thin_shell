#ifndef ANI_MESH_H
#define ANI_MESH_H

#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MPoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMeshData.h>
#include <maya/MPxCommand.h>

class AniMesh : public MPxNode
{
public:
  AniMesh() {};
  virtual         ~AniMesh() {};
  virtual MStatus compute(const MPlug& plug, MDataBlock& data);
  static  void*   creator();
  static  MStatus initialize();

  static MObject  time;
  static MObject  outputMesh;
  static MTypeId  id;

protected:
  MObject readFrame(const MTime& time, MObject& outData, MStatus& stat);
};

#endif /* ANI_MESH_H */
