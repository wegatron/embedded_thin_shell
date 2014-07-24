#ifndef __LAPLACIAN_OPERATOR_H__
#define __LAPLACIAN_OPERATOR_H__

#include <Eigen/Sparse>
#include <zjucad/matrix/matrix.h>
#include <jtflib/mesh/mesh.h>
#include <hjlib/sparse/sparse.h>

using namespace std;
using namespace zjucad::matrix;
using namespace Eigen;
using namespace jtf::mesh;
using namespace hj::sparse;

/**
 * @brief The laplacian_operator class : only used for boundless triangle mesh!
 */
class laplacian_operator {
public :
    laplacian_operator(const matrixst &tris, const matrixd &nods)
        : tris_(tris), nods_(nods) {
        p_e2c_.reset(edge2cell_adjacent::create(tris_, false));

        p_p2p_.reset(one_ring_point_at_point::create(tris_));
        p_p2p_->sort_into_loop(tris_, *p_e2c_);

        p_p2f_.reset(new one_ring_face_at_point);
        p_p2f_->add_all_faces(tris_, *p_e2c_);
        p_p2f_->sort_int_loop(tris_, nods_);

        cal_all_one_ring_area();
    }

    int construct_lap_matrix(Eigen::SparseMatrix<double> &L) {
        for (auto it = p_p2p_->p2p_.begin(); it != p_p2p_->p2p_.end(); ++it) {
            size_t pid = it->first;
            double weight_sum = 0;
            size_t N = it->second.size() - 1;
            for (int pi = 0; pi < N; ++pi) {
                size_t qid = it->second[pi];
                size_t last = it->second[(pi - 1) % N];
                size_t next = it->second[(pi + 1) % N];
                double cotA = cal_angle_cot_val(nods_(colon(), pid), nods_(colon(), last), nods_(colon(), qid));
                double cotB = cal_angle_cot_val(nods_(colon(), pid), nods_(colon(), next), nods_(colon(), qid));
                double w_pq = -0.5 * sqrt(3 / one_ring_area_[pid]) * ( cotA + cotB );
                weight_sum += w_pq;
                for (int offset = 0; offset < 3; ++offset)
                    ele_L_.push_back(Triplet<double>(pid * 3 + offset, qid * 3 + offset, w_pq));
            }
            for (int offset = 0; offset < 3; ++offset)
                ele_L_.push_back(Triplet<double>(pid * 3 + offset, pid * 3 + offset, -weight_sum));
        }

        L.resize(nods_.size(), nods_.size());
        L.reserve(ele_L_.size());
        L.setFromTriplets(ele_L_.begin(), ele_L_.end());
        L.makeCompressed();

        return 0;
    }

private :
    /**
     * @brief calculate the area of triangle face fid
     */
    double cal_face_area(const size_t fid) {
        matrixd verts = nods_(colon(), tris_(colon(), fid));
        matrixd edges = verts(colon(), colon(1, 2)) - verts(colon(), colon(0, 1));
        return 0.5 * fabs(norm(cross(edges(colon(), 0), edges(colon(), 1))));
    }

    /**
     * @brief calculate the cotan value of anlge ABC
     */
    double cal_angle_cot_val(const matrixd &A, const matrixd &B, const matrixd &C) {
        matrixd e0 = A - B;
        matrixd e1 = C - B;
        return 1.0 / tan(acos(dot(A, B) / norm(A) / norm(B)));
    }

    int cal_all_one_ring_area() {
        one_ring_area_ = zeros<double>(nods_.size(2));
        for (auto it = p_p2f_->p2f_.begin(); it != p_p2f_->p2f_.end(); ++it) {
            size_t pid = it->first;
            for (size_t fi = 0; fi < it->second.size() - 1; ++fi)
                one_ring_area_[pid] += cal_face_area(it->second[fi]);
        }
        return 0;
    }

private :
    const matrixst                       &tris_;
    const matrixd                        &nods_;

    shared_ptr<edge2cell_adjacent>       p_e2c_;
    shared_ptr<one_ring_point_at_point>  p_p2p_;
    shared_ptr<one_ring_face_at_point>   p_p2f_;

    matrixd                              one_ring_area_;
    vector<Eigen::Triplet<double>>       ele_L_;
};


















#endif
