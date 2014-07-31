#include <string> 
#include <sys/types.h>
#include <maya/MStatus.h>
#include <maya/MPxCommand.h>
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
#include <maya/MFnPlugin.h>
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
#include <fstream>
#include <iostream>
#include <maya/MSimple.h>
#include <assert.h>
#include "refine_softbody.h"

using namespace std;
DeclareSimpleCommand( RefineSoftBody, "kongll", "2013.5");


MStatus RefineSoftBody::doIt(const MArgList& args) {
	MStatus status;
	
	MIntArray vertex_count;
	MIntArray vertex_list;
	MPointArray points;

	MTime time;
	MAnimControl animctrl;
	double startframe= args.asDouble(0,&status);
	double endframe=args.asDouble(1,&status);
	double totaltime=endframe-startframe+1;
	time = startframe;
	for (size_t frame=0;frame<totaltime;frame++)
	{
		animctrl.setCurrentTime(time);
		status = ExtractSelect(time, vertex_count, vertex_list, points);
		char filename[256];
		sprintf(filename,"C:\\Users\\Administrator\\Desktop\\plugin\\OBJ\\maya_export%03d.obj",frame);
		status=ExportObj(filename,vertex_count,vertex_list,points);
		time++;
	}
	return status;
}

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
	assert(fp);
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

//MStatus ImportObjs (const string prefix, const size_t count){
//	MStatus status;
//	return status;
//}
//MStatus Refine (const MIntArray &vertex_count, const MIntArray &vertex_list, const MPointArray &points, const string &prefix, const size_t count){
//	MStatus status;
//	return status;
//}
