#ifndef CJ_CLUSTER_H_
#define CJ_CLUSTER_H_

#include <map>
#include <zjucad/matrix/matrix.h>
#include <boost/unordered_set.hpp>

class cluster_machine {

typedef struct {
    size_t        id, center;
    double        dis;
    std::string   path;
}state;

public :
    cluster_machine(const zjucad::matrix::matrix<size_t>  &mesh,
                    const zjucad::matrix::matrix<double>  &nodes,
                    const double                          R)
        :mesh_(mesh), nodes_(nodes), R_(R) {
//        remove_extra_node();
        prime_.resize(nodes_.size(2));
        auxil_.resize(nodes_.size(2));
    }
    int       partition(const size_t num);

private :
    int       remove_extra_node();
    int       build_adj_list();
    int       bfs(const size_t start);
    bool      stop();
    bool      update(size_t &next);
    void      dfs(const size_t curr, boost::unordered_set<size_t> &vis);
    size_t    connected_component();

public :
    std::map<size_t, std::vector<std::pair<size_t, std::string>>>  regions_;
    const zjucad::matrix::matrix<size_t>       &mesh_;
    const zjucad::matrix::matrix<double>       &nodes_;
public :
    const double                         R_;
    std::vector<size_t>                  u_;
    std::vector<size_t>                  v_;
    std::vector<size_t>                  first_;
    std::vector<size_t>                  next_;

    std::vector<state>                   prime_;
    std::vector<state>                   auxil_;

};

int calc_dist_to_center(const zjucad::matrix::matrix<double>                                 &emb_node,
                        const std::map<size_t, std::vector<std::pair<size_t, std::string>>>  &regions,
                        std::map<size_t, std::vector<std::pair<size_t, double>>>             &region_dis);


int calc_point_weight(const zjucad::matrix::matrix<double>                                 &emb_node,
                      const std::map<size_t, std::vector<std::pair<size_t, std::string>>>  &regions,
                      std::vector<std::vector<std::pair<size_t, double>>>                  &clt_w);


#endif // CLUSTER_H
