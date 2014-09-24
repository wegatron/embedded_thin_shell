#include "executor.h"

#include <maya/MFnPlugin.h>

#include "ani_mesh.h"

MObject AniMesh::time;
MObject AniMesh::outputMesh;
MTypeId AniMesh::id( 0x80000 );

//MStatus initializePlugin(MObject obj)
//{
//	MStatus   status;
//	MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "WE");
//	status = plugin.registerNode("AniMesh", AniMesh::id,AniMesh::creator, AniMesh::initialize);
//	status = plugin.registerCommand("import_vrml", Importer::Execute);
//	status = plugin.registerCommand("export_vrml", Exporter::Execute);
//	if (!status) {
//		status.perror("registerNode");
//		return status;
//	}
//	return status;
//}
//
//MStatus uninitializePlugin(MObject obj)
//{
//	MStatus   status;
//	MFnPlugin plugin(obj);
//
//	status = plugin.deregisterNode(AniMesh::id);
//	status = plugin.deregisterCommand("import_vrml");
//	status = plugin.deregisterCommand("export_vrml");
//	if (!status) {
//		status.perror("deregisterNode");
//		return status;
//	}
//	return status;
//}

//MStatus Excute() {
//  // export data in frame 0
//  // init embedded shell simulator
//  // for (i=1 to n) {
//  //  forward and export vrml file
//  // }
//  // load and create node
//  return MS::kSuccess; 
//}
//
