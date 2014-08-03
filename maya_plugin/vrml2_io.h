#ifndef VRML2_IO_H
#define VRML2_IO_H

#include <vector>
#include <string>

int ExportVrml2 (const std::string& filename, const std::vector<size_t> &face_v, const std::vector<double> &points_v);

int ImportVrml2 (const std::string& filename, std::vector<size_t> &face_v, std::vector<double> &points_v);

#endif /* VRML2_IO_H */
