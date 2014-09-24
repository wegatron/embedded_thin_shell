#ifndef MAYA_OP_H
#define MAYA_OP_H

#include <string>
#include <vector>
#include <sys/types.h>
#include <maya/MTime.h>
#include <maya/MStatus.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MFnDagNode.h>

#include "maya_plug_setting.h"


// extract the selected mesh in the specific time.
MStatus ExtractSelect (const MTime &time, MIntArray &vertex_count, MIntArray &vertex_list, MPointArray &points);

// export the mesh to a obj file
MStatus ExportObj (const char *file_name, const MIntArray &vertex_count, const MIntArray &vertex_list, const MPointArray &points);

MStatus ImportVrml();

MStatus ExportSelect2Vrml(const size_t start_frame, const size_t end_frame, const std::string &file_prefix);

// do remove the old files and refine in new thread windows version
HANDLE DoRefine(const std::string &strExpPerfix, const std::string &strImpPerfix, const int sub, const int start, const int end, int &all_res);

// check the file of current_time is exist, and set the current time to show the result
void LoadUpdate(const std::string &strImpPerfix, const int current_time, const int *res);

#endif /* REFINE_SOFTBODY_H */
