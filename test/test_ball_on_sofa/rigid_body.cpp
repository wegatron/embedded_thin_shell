#include "rigid_body.h"

#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include <sstream>


using namespace std;

rigid_bodys::rigid_bodys(const double gravity, const double dt)
    : _gravity(gravity), _dt(dt), _rigidImpulse(0.0, 0.0, 0.0)
{
    init_dynamics_world();
}

rigid_bodys::~rigid_bodys()
{
   //remove the rigidbodies from the dynamics world and delete them
    assert(_dynamicsWorld);
    for ( int i=_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i-- )
    {
        btCollisionObject* obj = _dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        _dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    for (int j=0; j<_collisionShapes.size(); j++ )
    {
        btCollisionShape* shape = _collisionShapes[j];
        _collisionShapes[j] = 0;
        delete shape;
    }

    delete _dynamicsWorld;
    delete _solver;
    delete _overlappingPairCache;
    delete _dispatcher;
    delete _collisionConfiguration;
}

void rigid_bodys::init_dynamics_world()
{
    assert(_collisionConfiguration = new btDefaultCollisionConfiguration());
    assert(_dispatcher = new  btCollisionDispatcher(_collisionConfiguration));
    assert(_overlappingPairCache = new btDbvtBroadphase());
    assert(_solver = new btSequentialImpulseConstraintSolver);

    assert(_dynamicsWorld = new btDiscreteDynamicsWorld(_dispatcher,_overlappingPairCache,_solver,_collisionConfiguration));
    _dynamicsWorld->setGravity(btVector3(0, -_gravity, 0));
}

void rigid_bodys::step()
{
    for(int i=0;i<_bodies.size();i++)
    {
      btRigidBody* body=_bodies[i];
      body->applyCentralImpulse(_rigidImpulse);
    }
    _dynamicsWorld->stepSimulation(_dt, 10);
}


void rigid_bodys::add_static_horizontal_plane(const PlaneInfo &info)
{
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(info.pos);

    btCollisionShape* boxShape = new btBoxShape(btVector3(info.width, info.height, info.thickness));
    assert(boxShape);
    _collisionShapes.push_back(boxShape);
    add_rigid_body(0.0, startTransform, boxShape);
}

int rigid_bodys::add_rigid_body(const zjucad::matrix::matrix<size_t> &mesh,
                                const zjucad::matrix::matrix<double> &nodes,
                                RigidBodyInfo                        &info)
{
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(info.pos);

    btTriangleMesh *trimesh = new btTriangleMesh();
    _trimesh.push_back(trimesh);

    for (size_t i = 0; i < mesh.size(2); ++i) {
        btVector3 v0(nodes(0, mesh(0, i)), nodes(1, mesh(0, i)), nodes(2, mesh(0, i)));
        btVector3 v1(nodes(0, mesh(1, i)), nodes(1, mesh(1, i)), nodes(2, mesh(1, i)));
        btVector3 v2(nodes(0, mesh(2, i)), nodes(1, mesh(2, i)), nodes(2, mesh(2, i)));

        v0 *= info.scale;
        v1 *= info.scale;
        v2 *= info.scale;

        _trimesh->addTriangle(v0, v1, v2);
    }

    shared_ptr<btConvexHullShape> tmpConvexShape(new btConvexTriangleMeshShape(_trimesh));
    shared_ptr<btShapeHull> hull(new btShapeHull(tmpConvexShape));
    btScalar margin = tmpConvexShape->getMargin();
    hull->buildHull(margin);
    tmpConvexShape->setUserPointer(hull);


    _collisionShapes.puch_back();
    btRigidBody body = add_rigid_body(info.mass, startTransform, SHAPE);
    _bodies.puch_back(body);

    return _bodies.size() - 1;
}

int rigid_bodys::addRigidBody(const WaveFrontObjMesh & renderMesh,
                             const WaveFrontObjMesh & phyMesh, RigidBodyInfo & info)

{
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(info.pos);
  // add rigid body
  {
    // build triangle mesh first
    btTriangleMesh* trimesh = new btTriangleMesh();
    _trimeshes.push_back(trimesh);

    btTriangleMesh* trimeshForPhy = new btTriangleMesh();
    _trimeshesForPhy.push_back(trimeshForPhy);

    SizeType nTri = renderMesh.getNumTriangles();
    INFO("Rigidbody's triangle number for render : %lu",nTri);
    for ( SizeType i=0; i < nTri; i++ )
    {
      const WaveFrontObjMesh::Triangle tri = renderMesh.getTriangle(i);

      btVector3 vertex0(tri.vertices[0].x, tri.vertices[0].y, tri.vertices[0].z);
      btVector3 vertex1(tri.vertices[1].x, tri.vertices[1].y, tri.vertices[1].z);
      btVector3 vertex2(tri.vertices[2].x, tri.vertices[2].y, tri.vertices[2].z);

      vertex0 *= info.scale;
      vertex1 *= info.scale;
      vertex2 *= info.scale;

      trimesh->addTriangle(vertex0,vertex1,vertex2);
    }

    SizeType nTriForPhy = phyMesh.getNumTriangles();
    INFO("Rigidbody's triangle number for simulation: %lu",nTriForPhy);
    for ( SizeType i=0; i < nTriForPhy; i++ )
    {
      const WaveFrontObjMesh::Triangle tri = phyMesh.getTriangle(i);

      btVector3 vertex0(tri.vertices[0].x, tri.vertices[0].y, tri.vertices[0].z);
      btVector3 vertex1(tri.vertices[1].x, tri.vertices[1].y, tri.vertices[1].z);
      btVector3 vertex2(tri.vertices[2].x, tri.vertices[2].y, tri.vertices[2].z);

      vertex0 *= info.scale;
      vertex1 *= info.scale;
      vertex2 *= info.scale;

      trimeshForPhy->addTriangle(vertex0,vertex1,vertex2);
    }

    // create a hull approximation
    btConvexShape* tmpConvexShape = new btConvexTriangleMeshShape(trimesh);
    btShapeHull* hull = new btShapeHull(tmpConvexShape); // reduced model
    btScalar margin = tmpConvexShape->getMargin();
    hull->buildHull(margin);
    tmpConvexShape->setUserPointer(hull);

    btVector3 minBB,maxBB;
    tmpConvexShape->getAabb(startTransform,minBB,maxBB);
    btVector3 hullLen = maxBB - minBB;
    Real roughV = hullLen.x()*hullLen.y()*hullLen.z();
    info.mass *= roughV;
    INFO("RigidBody rough volume: %.2f", roughV);
    INFO("RigidBody mass: %.2f", info.mass);

    INFO("RigidBody BoundingBox : (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)",
         minBB.x(),minBB.y(),minBB.z(),maxBB.x(),maxBB.y(),maxBB.z());

    // printf("new numTriangles = %d\n", hull->numTriangles ());
    // printf("new numIndices = %d\n", hull->numIndices ());
    // printf("new numVertices = %d\n", hull->numVertices ());

    // create final rigid body shape used in bullet
    btConvexHullShape* convexShape = new btConvexHullShape();
    for ( SizeType i=0; i < hull->numVertices(); i++ )
    {
      convexShape->addPoint(hull->getVertexPointer()[i]);
    }
    delete tmpConvexShape;
    delete hull;
    _collisionShapes.push_back(convexShape);

    btRigidBody* body=add_rigid_body(info.mass, startTransform, convexShape);

    _bodies.push_back(body);
    _rotors.push_back(RigidBodyRotor());

    return _bodies.size()-1;
  }

}

void rigid_bodys::writeVTK(const btIndexedMesh& tm,const btTransform& t,const std::string& name,float len=0.1f)
{
    FILE * fp = fopen(name.c_str(), "w");
    assert(fp);

    fprintf(fp, "# vtk DataFile Version 1.0\n");
    fprintf(fp, "2D Unstructured Grid of Linear Triangles\n");
    fprintf(fp, "ASCII\n");
    fprintf(fp, "\nDATASET UNSTRUCTURED_GRID\n");
    fprintf(fp, "POINTS %d float\n", tm.m_numVertices+tm.m_numTriangles*2);
    std::vector<btVector3> vss;
    for ( SizeType i=0; i < tm.m_numVertices; i++ )
    {
        assert(PHY_FLOAT == tm.m_vertexType);

        btVector3 bv(
                    ((float*)(tm.m_vertexBase+tm.m_vertexStride*i))[0],
                    ((float*)(tm.m_vertexBase+tm.m_vertexStride*i))[1],
                    ((float*)(tm.m_vertexBase+tm.m_vertexStride*i))[2]);

        btVector3 bvOut=t*bv;
        vss.push_back(bvOut);
        fprintf(fp, "%f %f %f\n",bvOut.x(),bvOut.y(),bvOut.z());
    }

    for( SizeType i=0; i < tm.m_numTriangles; i++ )
    {
        assert(PHY_INTEGER == tm.m_indexType);

        unsigned int a=((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[0];
        unsigned int b=((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[1];
        unsigned int c=((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[2];

        btVector3 n0=(vss[a]+vss[b]+vss[c])/3.0f;
        btVector3 n=(vss[c]-vss[a]).cross(vss[b]-vss[a]).normalized()*len;
        btVector3 n1=n0+n;
        fprintf(fp, "%f %f %f\n",n0.x(),n0.y(),n0.z());
        fprintf(fp, "%f %f %f\n",n1.x(),n1.y(),n1.z());
    }

    fprintf(fp, "\n");
    fprintf(fp, "CELLS %d %d\n", tm.m_numTriangles*2,tm.m_numTriangles*(4+3));

    for( SizeType i=0; i < tm.m_numTriangles; i++ )
    {
        assert(PHY_INTEGER == tm.m_indexType);
        fprintf(fp, "%d %d %d %d\n",
                3,
                ((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[0],
                ((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[1],
                ((unsigned int*)(tm.m_triangleIndexBase+tm.m_triangleIndexStride*i))[2]);
    }

    for( SizeType i=0,j=tm.m_numVertices; i < tm.m_numTriangles; i++ , j+=2)
    {
        assert(PHY_INTEGER == tm.m_indexType);
        fprintf(fp, "%d %lu %lu\n",2,j,j+1);
    }
    fprintf(fp, "\n");
    fprintf(fp, "CELL_TYPES %d\n", tm.m_numTriangles*2);
    for( SizeType i=0; i < tm.m_numTriangles; i++ )
    {
        fprintf(fp, "5\n");
    }
    for( SizeType i=0; i < tm.m_numTriangles; i++ )
    {
        fprintf(fp, "3\n");
    }
    fprintf(fp, "\n");
    fclose(fp);
}

void rigid_bodys::dump(const std::string& name, int f) const
{
    int nr=_bodies.size();
    for(int i=0;i < nr;i++)
    {
        btCollisionObject* obj=_bodies[i];
        btTransform& t=obj->getWorldTransform();
        btTriangleMesh* tm=_trimesh[i];

        IndexedMeshArray& arr=tm->getIndexedMeshArray();
        for(int j=0;j<arr.size();j++)
        {
            std::ostringstream oss;
            oss << name << "_" << i << "_" << j << "_" << f << ".vtk";
            writeVTK(arr[j],t,oss.str());
        }
    }
}

btRigidBody* rigid_bodys::add_rigid_body(const double mass, const btTransform &startTransform, btCollisionShape *shape, int id)
{
    assert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
    body->setSleepingThresholds(0.0f,0.0f);
    body->setUserPointer((void*)id);
    _dynamicsWorld->addRigidBody(body);

    return body;
}
