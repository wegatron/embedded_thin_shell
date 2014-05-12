#ifndef __CJ_METIS_DECOMPOSITION_H__
#define __CJ_METIS_DECOMPOSITION_H__

#include <metis.h>
#include <zjucad/matrix/matrix.h>
#include <boost/unordered_map.hpp>

using namespace std;
using zjucad::matrix::matrix;

struct Patch {
    matrix<size_t>                        mesh_;
    matrix<double>                        nodes_;
    vector<vector<pair<size_t, double>>>  cons_;
    boost::unordered_map<size_t, size_t>  g2s_;
    boost::unordered_map<size_t, size_t>  s2g_;
};

typedef shared_ptr<Patch> pPatch;

class decomposition
{
public :
    decomposition(const matrix<size_t>  &mesh,
                  const matrix<double>  &nodes,
                  const size_t          threadsNum)
        : mesh_(mesh), nodes_(nodes), threadsNum_(threadsNum) { }

    int generatePatches(vector<pPatch> &group);
    int writeAllPatches(const char           *dirPath,
                        const char           *filePrefix,
                        const vector<pPatch> &group);
private :
    int genSubMesh(vector<pPatch> &group);
    int genConstraint(vector<pPatch> &group);
    int postProcessing(const vector<idx_t>  &epart,
                       const vector<idx_t>  &npart,
                       vector<pPatch>       &group);
    int postProcessing(const vector<idx_t>  &epart,
                       vector<pPatch>       &group);

private :
    const matrix<size_t>  &mesh_;
    const matrix<double>  &nodes_;
    const size_t          threadsNum_;
};
#endif
