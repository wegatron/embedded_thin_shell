#ifndef REFINE_SOFTBODY_H
#define REFINE_SOFTBODY_H
#include <vector>
#include <maya/MFnDagNode.h>

#define LOG_FILE "c:\\maya_refine_softbody_log.txt"

//#define JUDGE_LOG(ostream, msg, status) do {      \
//	if(status!=MStatusCode::kSuccess) { \
//	ostream << msg << endl; \
//	} \
//}while(0)

// extract the selected mesh in the specific time.
MStatus ExtractSelect (const MTime &time, MIntArray &vertex_count, MIntArray &vertex_list, MPointArray &points);
// export the mesh to a obj file
MStatus ExportObj (const char *file_name, const MIntArray &vertex_count, const MIntArray &vertex_list, const MPointArray &points);

// import a obj squence start from 1 like mesh001.obj mesh002.obj
// bite is the min lenth of the number
//MStatus ImportObjs (const string prefix, const size_t bite, const size_t count);

// refine the data and out put to the obj file squence
//MStatus Refine (const MIntArray &vertex_count, const IntArray &vertex_list, const MPointArray &points, const string &prefix, const size_t count);

#endif /* REFINE_SOFTBODY_H */
