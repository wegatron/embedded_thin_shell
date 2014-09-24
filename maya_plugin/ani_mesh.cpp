#include "ani_mesh.h"

#include <stdio.h>
#include <string.h>

#include <maya/MIOStream.h>

#include "maya_plug_setting.h"
#include "vrml2_io.h"
#include <iostream>
#include <fstream>
using namespace std;

static string import_prefix;

void setImportPrefix(const string &prefix)
{
	import_prefix = prefix;
}

MStatus returnStatus;

#define McheckErr(stat,msg)         \
	if ( MS::kSuccess != stat ) {   \
	cerr << msg;                \
	return MS::kFailure;        \
	}


void* AniMesh::creator()
{
  return new AniMesh;
}

MStatus AniMesh::initialize()
{
	MFnUnitAttribute unitAttr;
	MFnTypedAttribute typedAttr;

	MStatus returnStatus;

	AniMesh::time = unitAttr.create( "time", "tm",
		MFnUnitAttribute::kTime,
		0.0, &returnStatus );
	McheckErr(returnStatus, "ERROR creating AniMesh time attribute\n");
	AniMesh::outputMesh = typedAttr.create( "outputMesh", "out",
		MFnData::kMesh,
		&returnStatus ); 
	McheckErr(returnStatus, "ERROR creating AniMesh output attribute\n");
	typedAttr.setStorable(false);

	returnStatus = addAttribute(AniMesh::time);
	McheckErr(returnStatus, "ERROR adding time attribute\n");

	returnStatus = addAttribute(AniMesh::outputMesh);
	McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

	returnStatus = attributeAffects(AniMesh::time,
		AniMesh::outputMesh);
	McheckErr(returnStatus, "ERROR in attributeAffects\n");

	return MS::kSuccess;
}

MObject AniMesh::readFrame(const MTime& time,MObject& outData,MStatus& stat)

{
	MFloatPointArray points;
	MFnMesh         meshFS;
	int frame = (int)time.as( MTime::kFilm );
	if (frame == 0)
		frame = 1; 
	vector<size_t> face_v;
	vector<double> points_v;
	char cfilename[256];
	sprintf(cfilename, "%s%d.vrml",import_prefix.c_str(),frame);
	//sprintf(cfilename, "%s%d.vrml",import_prefix.c_str(),0);
	string filename = string(cfilename);
	fstream fp;
	fp.open(filename,ios::in);
	if (fp)
	{
		ImportVrml2 (filename, face_v, points_v);
	}else{
		sprintf(cfilename, "%s%d.vrml",import_prefix.c_str(),0);
		string filename = string(cfilename); 
		ImportVrml2(filename,face_v,points_v);
	}
	
	size_t numVertices = points_v.size()/3;
	size_t numFaces = face_v.size()/3;
	for(vector<double>::const_iterator it = points_v.begin();it != points_v.end();it+=3) {
		MFloatPoint vtx(*it,*(it+1),*(it+2));
		points.append(vtx);
	}

	vector<int> face_count;
	for(int i=0;i<numFaces;i++) {
		face_count.push_back(3);
	}
	MIntArray faceCounts(&face_count[0],numFaces);

	vector<int> face_connects;
	face_connects.resize(face_v.size());
	for(int i=0;i<face_v.size();++i)
	{
		face_connects[i] = face_v[i];
	}

	MIntArray faceConnects( &face_connects[0], face_connects.size() );
	MObject newMesh=meshFS.create(numVertices, numFaces,points, faceCounts, faceConnects,outData,&stat);
	return newMesh;

}

MStatus AniMesh::compute(const MPlug& plug, MDataBlock& data)

{
	MStatus returnStatus;

	if (plug == outputMesh) {
		/* Get time */
		MDataHandle timeData = data.inputValue( time, &returnStatus ); 
		McheckErr(returnStatus, "Error getting time data handle\n");
		MTime time = timeData.asTime();
		/* Get output object */

		MDataHandle outputHandle = data.outputValue(outputMesh, &returnStatus);
		McheckErr(returnStatus, "ERROR getting polygon data handle\n");

		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&returnStatus);
		McheckErr(returnStatus, "ERROR creating outputData");

		readFrame(time, newOutputData, returnStatus);
		McheckErr(returnStatus, "ERROR creating new Cube");
		outputHandle.set(newOutputData);
		data.setClean( plug );
	} else
		return MS::kUnknownParameter;

	return MS::kSuccess;
}
