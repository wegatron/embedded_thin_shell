#include "metis_decomposition.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <metis.h>
#include <set>
#include "src/shell_deformer/cluster.h"
#include "src/conf_para.h"
#include "common/vtk.h"

using namespace zjucad::matrix;

int decomposition::generatePatches(vector<pPatch> &group)
{
    cout << "[INFO] partioning whole mesh into small pieces\n";
    if ( genSubMesh(group) )
        return __LINE__;
    cout << "[INFO] add constraints on each pieces\n";
    if ( genConstraint(group) )
        return __LINE__;
    return 0;
}

int decomposition::genSubMesh(vector<pPatch> &group)
{
    group.resize(threadsNum_);
    for (size_t i = 0; i < group.size(); ++i)
        group[i].reset(new Patch);

    vector<idx_t> eptr(mesh_.size(2) + 1);
    vector<idx_t> eind(mesh_.size());
    std::copy(mesh_.begin(), mesh_.end(), eind.begin());
    for (idx_t i = 0; i < eptr.size(); ++i)
        eptr[i] = 3 * i;
    idx_t ne = mesh_.size(2);
    idx_t nn = nodes_.size(2);
    idx_t nparts = threadsNum_;
    idx_t objval;
    vector<idx_t> epart(ne);
    vector<idx_t> npart(nn);

    if ( nparts > 1 )
        METIS_PartMeshNodal(&ne, &nn, &eptr[0], &eind[0], NULL, NULL,
                            &nparts, NULL, NULL, &objval, &epart[0], &npart[0]);
    else {
        for (size_t i = 0; i < epart.size(); ++i)
            epart[i] = 0;
    }

    cout << "[INFO] raw partition data has been generated\n";
    postProcessing(epart, group);

    return 0;
}

int decomposition::postProcessing(const vector<idx_t> &epart,
                                  vector<pPatch>      &group)
{
    vector<vector<size_t>> patchCell(group.size());
    for (size_t i = 0; i < epart.size(); ++i)
        patchCell[epart[i]].push_back(i);

#pragma omp parallel for
    for (size_t i = 0; i < group.size(); ++i) {
        set<size_t> nodeIdx;
        for (size_t j = 0; j < patchCell[i].size(); ++j) {
            size_t cellId = patchCell[i][j];
            nodeIdx.insert(mesh_(0, cellId));
            nodeIdx.insert(mesh_(1, cellId));
            nodeIdx.insert(mesh_(2, cellId));
        }
        group[i]->mesh_.resize(3, patchCell[i].size());
        group[i]->nodes_.resize(3, nodeIdx.size());
        size_t col = 0;
        for (auto it = nodeIdx.begin(); it != nodeIdx.end(); ++it, ++col) {
            group[i]->nodes_(colon(), col) = nodes_(colon(), *it);
            group[i]->g2s_.insert(make_pair(*it, col));
            group[i]->s2g_.insert(make_pair(col, *it));
        }
        for (size_t j = 0; j < patchCell[i].size(); ++j) {
            size_t fid = patchCell[i][j];
            for (size_t k = 0; k < 3; ++k)
                group[i]->mesh_(k, j) = group[i]->g2s_[mesh_(k, fid)];
        }
    }
    return 0;
}

int decomposition::postProcessing(const vector<idx_t> &epart,
                                  const vector<idx_t> &npart,
                                  vector<pPatch>      &group)
{
    vector<vector<size_t>> patchCell(group.size());
    vector<vector<size_t>> patchNode(group.size());
    for (size_t i = 0; i < epart.size(); ++i)
        patchCell[epart[i]].push_back(i);
    for (size_t i = 0; i < npart.size(); ++i)
        patchNode[npart[i]].push_back(i);

    for (size_t i = 0; i < group.size(); ++i) {
        group[i]->mesh_.resize(3, patchCell[i].size());
        group[i]->nodes_.resize(3, patchNode[i].size());
        for (size_t j = 0; j < patchNode[i].size(); ++j) {
            group[i]->nodes_(colon(), j) = nodes_(colon(), patchNode[i][j]);
            group[i]->g2s_.insert(make_pair(patchNode[i][j], j));
            group[i]->s2g_.insert(make_pair(j, patchNode[i][j]));
        }
        for (size_t j = 0; j < patchCell[i].size(); ++j) {
            size_t fid = patchCell[i][j];
            for (size_t k = 0; k < 3; ++k)
                group[i]->mesh_(k, j) = group[i]->g2s_[mesh_(k, fid)];
        }
    }
    return 0;
}

int decomposition::genConstraint(vector<pPatch> &group)
{
#pragma omp parallel for
    for (size_t i = 0; i < group.size(); ++i) {
        assert(group[i]->mesh_.size() > 0 && group[i]->nodes_.size() > 0);
        shared_ptr<cluster_machine> pCM(new cluster_machine(group[i]->mesh_,
                                                            group[i]->nodes_,
                                                            __CLUSTER_RADIUS));
        pCM->partition(static_cast<int>(__REGION_COUNT / threadsNum_));
        calc_point_weight(group[i]->nodes_, pCM->regions_, group[i]->cons_);
    }
    return 0;
}

int decomposition::writeAllPatches(const char           *dirPath,
                                   const char           *filePrefix,
                                   const vector<pPatch> &group)
{
#pragma omp parallel for
    for (size_t i = 0; i < group.size(); ++i) {
        stringstream curr;
        curr << dirPath << filePrefix << "_" << i << ".vtk";
        string outfile = curr.str();
        ofstream out(outfile.c_str());
        tri2vtk(out, &group[i]->nodes_[0], group[i]->nodes_.size(2),
                &group[i]->mesh_[0], group[i]->mesh_.size(2));
    }
    return 0;
}
