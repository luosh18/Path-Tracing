#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#define Pepsilon (1e-25)

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {

    }

    Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
        this->offset = d / normal.length(); // normalizing leads to easier calculation
        this->normal = normal.normalized(); 
    }

    ~Plane() override = default;

    // https://math.stackexchange.com/questions/1979876/ray-casting-algorithm-in-ray-triangle-intersection
    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float check = Vector3f::dot(r.getDirection(), normal);
        if (abs(check) > Pepsilon) {
            float t = (offset - Vector3f::dot(r.getOrigin(), normal)) / check;
            if (t>tmin) {
                h.set(t, material, normal);
                return true;
            }
        }
        return false;
    }

protected:
    Vector3f normal;    // (a, b, c)    //normalized
    float offset;       // d            //normalized

};

#endif //PLANE_H
		

