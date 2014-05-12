#ifndef __CJ_MESH_OPERATION_H__
#define __CJ_MESH_OPERATION_H__

#include <zjucad/matrix/matrix.h>
#include <map>
#include <set>

using namespace std;

inline void remove_extra_node(zjucad::matrix::matrix<size_t> & cell,
                              zjucad::matrix::matrix<double> & node,
                              zjucad::matrix::matrix<size_t> * mapping = 0)
{
    typedef zjucad::matrix::matrix<size_t> matrixst;
    typedef zjucad::matrix::matrix<double> matrixd;
    using namespace std;
    using namespace zjucad::matrix;
    set<size_t> used_node_idx(cell.begin(), cell.end());
    if(used_node_idx.size() == node.size(2)) {
//        cout << "[INFO] no redundant nodes\n";
        return;
    }
    matrixst used_node_mat(used_node_idx.size(),1);
    copy(used_node_idx.begin(), used_node_idx.end(), used_node_mat.begin());

    map<size_t,size_t> p2p;

    matrixd new_node(3, used_node_mat.size());
    for(size_t pi = 0; pi < used_node_mat.size(); ++pi){
        new_node(colon(),pi) = node(colon(), used_node_mat[pi]);
        p2p[used_node_mat[pi]] = pi;
    }
    for(size_t pi = 0; pi < cell.size(); ++pi)
        cell[pi] = p2p[cell[pi]];

    if(mapping != 0){
        *(mapping) = zjucad::matrix::ones<size_t>(node.size(2),1) * -1;
        for(map<size_t,size_t>::const_iterator cit = p2p.begin(); cit != p2p.end();
            ++cit){
            (*mapping)[cit->first] = cit->second;
        }
    }
    node = new_node;
}

#endif
