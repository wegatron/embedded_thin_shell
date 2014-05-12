#ifndef __CJ_RIGID_BODYS_H__
#define __CJ_RIGID_BODYS_H__

#include <zjucad/matrix/matrix.h>
#include "btBulletDynamicsCommon.h"


class rigid_bodys
{
public :
    struct RigidBodyInfo {
        double    scale;
        double    mass;
        btVector3 pos;
        RigidBodyInfo(double s, double m, double x, double y, double z)
            : scale(s), mass(m), pos(x, y, z) {
        }
    };
    struct PlaneInfo {
        double    width;
        double    height;
        double    thickness;
        btVector3 pos;
        PlaneInfo(double w, double h)
            : width(w), height(h), thickness(2), pos(0, 0, 0) { }
    };

public :
    rigid_bodys(const double gravity, const double dt);

    ~rigid_bodys();

    void step();

    void add_static_horizontal_plane(const PlaneInfo &info);

    int add_rigid_body(const zjucad::matrix::matrix<size_t> &mesh,
                       const zjucad::matrix::matrix<double> &nodes,
                       RigidBodyInfo                        &info);



    void set_impulse(const btVector3 im);

    void dump(const std::string &name, int f) const;

    void dump_ball(const btIndexedMesh &tm, const btTransform &t,
                   std::vector<btVector3> &p,
                   std::vector<btVector3> &f,
                   float len = 0.1f ) const;
private :

    void init_dynamics_world();

    btRigidBody* add_rigid_body(const double mass, const btTransform &starTransform,
                                btCollisionShape *shape, int id = 0);
private :
    btDefaultCollisionConfiguration       *_collisionConfiguration;
    btCollisionDispatcher                 *_dispatcher;
    btBroadphaseInterface                 *_overlappingPairCache;
    btSequentialImpulseConstraintSolver   *_solver;
    btDiscreteDynamicsWorld               *_dynamicsWorld;
    double                                 _gravity;
    double                                 _dt;
    btAlignedObjectArray<btCollisionShape*>       _collisionShapes;
    btAlignedObjectArray<btTriangleMesh*>         _trimesh;
    btAlignedObjectArray<btRigidBody*>            _bodies;
    btVector3                                     _rigidImpulse;

};











#endif
