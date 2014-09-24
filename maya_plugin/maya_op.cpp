#include "maya_op.h"

#include <io.h>
#include <stdio.h>
#include <stdlib.h>

#include <string> 
#include <fstream>
#include <iostream>
#include <sstream>

#include <sys/types.h>

#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSet.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshEdge.h>
#include <maya/MFloatVector.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFloatArray.h>
#include <maya/MObjectArray.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPxFileTranslator.h>
#include <maya/MItDag.h>
#include <maya/MDistance.h>
#include <maya/MIntArray.h>
#include <maya/MIOStream.h>
#include <maya/MAnimControl.h>


#include "convert.h"
#include "vrml2_io.h"

using namespace std;

MStatus ExtractSelect (const MTime &time, MIntArray &vertex_count, MIntArray &vertex_list, MPointArray &points)
{
	MStatus status;
	MSelectionList slist;
	MGlobal::getActiveSelectionList( slist );
	MItSelectionList iter( slist );

	if (iter.isDone()) {
		fprintf(stderr,"Error: Nothing is selected.\n");
		return MS::kFailure;
	}
	for ( ; !iter.isDone(); iter.next() )
	{
		MDagPath objectPath;
		status = iter.getDagPath( objectPath);
		MItDag dagIterator( MItDag::kDepthFirst, MFn::kInvalid, &status);

		status = dagIterator.reset (objectPath.node(), 
			MItDag::kDepthFirst, MFn::kInvalid );   
		for ( ; !dagIterator.isDone(); dagIterator.next() )
		{
			MDagPath dagPath;
			MObject  component = MObject::kNullObj;
			MSpace::Space space = MSpace::kWorld;
			status = dagIterator.getPath(dagPath);
			if (!status)
			{
				return MS::kFailure;
			}
			MFnDagNode dagNode(dagPath,&status);

			if(dagNode.isIntermediateObject()) continue;
			if(dagPath.hasFn(MFn::kTransform)) continue;
			if(dagPath.hasFn(MFn::kNurbsSurface))
			{
				status=MS::kSuccess;
			}
			if (dagPath.hasFn(MFn::kMesh))
			{
				MFnMesh fnmesh(dagPath);
				fnmesh.getPoints(points,space);	
				fnmesh.getVertices(vertex_count,vertex_list);
			}
		}
	}
	return status;
}

MStatus ExportObj (const char *file_name, const MIntArray &vertex_count, const MIntArray &vertex_list, const MPointArray &points){
	FILE* fp;
	fp=fopen(file_name,"w");
	if (fp==NULL)
	{
		fclose(fp);
		printf("can't open file for write !");
		exit(1);
	}
	for (size_t i=0;i<points.length();i++)
	{
		fprintf(fp,"v %f %f %f %f\n",points[i].x,points[i].y,points[i].z,points[i].w);
	}
	size_t vcount =0;
	for (size_t j=0;j<vertex_count.length();j++)
	{
		fprintf(fp,"f  ");
		for (size_t m=0;m<vertex_count[j];m++)
		{
			fprintf(fp,"%d  ",vertex_list[vcount]+1);
			vcount++;
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
	return MS::kSuccess;
}

MStatus ImportVrml()
{
  MString createNode1("createNode transform -n AniMesh;");
  createNode1+="createNode mesh -n AniMeshShape1 -p AniMesh;";
  createNode1+="createNode AniMesh -n AniMeshNode1;";
  createNode1+="connectAttr time1.outTime AniMeshNode1.time;";
  createNode1+="connectAttr AniMeshNode1.outputMesh AniMeshShape1.inMesh;";
  MGlobal::executeCommand(createNode1);
  return MS::kSuccess;
}

MStatus ExportSelect2Vrml(const size_t start_frame, const size_t end_frame, const string &file_prefix){
  MStatus status;
  MIntArray vertex_count;
  MIntArray vertex_list;
  MPointArray points;
  std::vector<size_t> faces_v;
  std::vector<double> points_v;

  MTime time;
  MAnimControl animctrl;
  size_t totaltime=end_frame-start_frame+1;
  time = (double) start_frame;
 
  for (size_t frame=start_frame; frame<=end_frame; frame++) {
    animctrl.setCurrentTime(time);
    status = ExtractSelect(time, vertex_count, vertex_list, points);
    char filename[266];
	char filename_type[256];
	strncpy(filename_type, file_prefix.c_str(), 240);
	strcat(filename_type, "%d.vrml");
    sprintf(filename, filename_type , frame);
    faces_v.resize(vertex_list.length());
    std::copy(&vertex_list[0], &vertex_list[0]+vertex_list.length(), &faces_v[0]);
    MAYA_PLUGIN::Convert(points, points_v);
    ExportVrml2(filename, faces_v, points_v);
    time++;
  }
 
  return MS::kSuccess;
}

struct ThreadData{
  string cmd;
  int res;
};

static DWORD WINAPI ExecCmd(LPVOID lpParam)
{
  ThreadData *data = (ThreadData *) lpParam;
  data->res = system(data->cmd.c_str());
  return 0;
}

HANDLE DoRefine(const string &strExpPerfix, const string &strImpPerfix, const int sub,
                const int start, const int end, int &all_res) {
  DWORD dwThreadId;
  HANDLE hThread;
  ThreadData data;

  data.res = -1; // executing
  std::stringstream ss;
  ss << "refine_shell.exe " << strExpPerfix << "0.vrml " << strExpPerfix << " " << strImpPerfix << " "
     << sub << " " << start << " " << end;
  data.cmd = ss.str();

  std::ofstream log("refine_cmd_log.txt", ofstream::out);
  log << data.cmd << endl;
  log.close();

  for (int i=start; i<=end; ++i) {
    std::stringstream ss;
    ss << strExpPerfix << i << ".vrml";
    remove(ss.str().c_str());

    ss.str(""); ss.clear();
    ss << strImpPerfix << i << ".vrml";
    remove(ss.str().c_str());
  }
  hThread = CreateThread(NULL,
                         0,
                         ExecCmd,
                         &data,
                         0,
                         &dwThreadId);
  return hThread;
}

void LoadUpdate(const string &strImpPerfix, const int current_time, const int *res)
{
  while(*res <= 0){
    std::stringstream ss;
    ss << strImpPerfix << current_time << ".vrml";
    if ( _access(ss.str().c_str(), 0) ) {
      break;
    }
	if (*res >0) { // error occurs
		return;
	}
  }

  MAnimControl animctrl;
  MTime time = (double) current_time;
  animctrl.setCurrentTime(time);
}

