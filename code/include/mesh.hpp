#ifndef MESH_H
#define MESH_H

#include <vector>
#include <algorithm>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

class AABB;
class KDNode;
class Mesh;
struct TriangleIndex;

// https://raytracing.github.io/books/RayTracingTheNextWeek.html#boundingvolumehierarchies/rayintersectionwithanaabb
class AABB : public Object3D {    // Axis-Aligned Bounding Boxes
public:
    AABB(const Vector3f &a, const Vector3f &b) {
        // Vector3f d = _max - _min;
        // _min = a - bias * d;
        // _max = b + bias * d;
        _min = a - AABBepsilon;
        _max = b + AABBepsilon;
    }

    int longestAxis() { // return the longest axis of the box
        float d[3];
        for (int i=0; i<3; i++) d[i] = fabs(_max[i] - _min[i]);
        int l = 0;
        for (int i=0; i<3; i++) if (d[i]>d[l]) l = i;
        return l;
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {  // do not edit hit!
        // if ray origin is inside the box, then it intersects
        bool inside = true;
        const Vector3f &orig = r.getOrigin();
        for (int i=0; i<3; i++) {
            if (orig[i] < _min[i] || orig[i] > _max[i]) {
                inside = false; break;
            }
        }
        if (inside) return true;
        // check outside ray
        float tmax = INFINITY;
        for (int a = 0; a < 3; a++) {
            auto invD = 1.0f / r.getDirection()[a];
            auto t0 = (_min[a] - r.getOrigin()[a]) * invD;
            auto t1 = (_max[a] - r.getOrigin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;
            if (tmax <= tmin)
                return false;
        }
        return true;
    }

    static const Vector3f AABBepsilon;
    static const float bias;
    Vector3f _min;
    Vector3f _max;
};

class KDNode : public Object3D {
public:
    KDNode(Mesh *mesh, std::vector<int> *list, int depth): mesh(mesh), list(list) {
		size = list->size();
        build(depth);
        // std::cout << depth << '\n';
    }
    ~KDNode() {
        if(list) delete list;
        if(left) delete left;
        if(right) delete right;
		if(box) delete box;
        if(triList) {
            for (int i=0; i<triList->size(); i++) {
                delete triList->at(i);
            }
            delete triList;
        }
    }
    bool x_comp(int a, int b);
    bool y_comp(int a, int b);
    bool z_comp(int a, int b);
    void build(int depth);
    bool intersect(const Ray &r, Hit &h, float tmin) override;

    static const int depth_bound = 60;
    Mesh *mesh;
    std::vector<int> *list = nullptr;
    int size;
    Object3D *left = nullptr;
    Object3D *right = nullptr;
    std::vector<Object3D*> *triList = nullptr;
    AABB *box = nullptr;
};

struct TriangleIndex {
    TriangleIndex() {
        x[0] = 0; x[1] = 0; x[2] = 0;
    }
    int &operator[](const int i) { return x[i]; }
    // By Computer Graphics convention, counterclockwise winding is front face
    int x[3]{};
};

class Mesh : public Object3D {

public:
    friend KDNode;
    Mesh(const char *filename, Material *m);

    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;
    bool intersect(const Ray &r, Hit &h, float tmin) override;

private:

    // Normal can be used for light estimation
    void computeNormal();
    KDNode *root;
};

#endif
