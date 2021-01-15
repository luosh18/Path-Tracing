// 0405 16:48

#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <algorithm>

// TODO: Implement functions and add more fields as necessary
// 0405 16:48 written

class Sphere : public Object3D {
public:
    Sphere() {
        // unit ball at the center
        center = Vector3f(0, 0, 0);
        radius = 1;
    }

    Sphere(const Vector3f &center, float radius, Material *material) : 
        Object3D(material), 
        center(center), 
        radius(radius) {
        // 
    }

    ~Sphere() override = default;

    // 0405 16:48
    bool intersect(const Ray &r, Hit &h, float tmin) override { // tmin 是下界 若t<=tmin 则没有相交
        // http://viclw17.github.io/2018/07/16/raytracing-ray-sphere-intersection/
        Vector3f r_orig = r.getOrigin();
        Vector3f r_dirc = r.getDirection();
        Vector3f oc = r_orig - center;
        float a = r_dirc.squaredLength();
        float b = 2.0 * Vector3f::dot(oc, r_dirc);
        float c = oc.squaredLength() - radius * radius;
        float discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0) {                              // 判别式小于0则没有相交
            float t1 = (-b - sqrt(discriminant)) / (2.0 * a);
            float t2 = (-b + sqrt(discriminant)) / (2.0 * a);
            float t;
            if (t1<=0 || t2<=0) t = std::max(t1, t2);  //// 已经修正 折射需要球体内相交
            else t = std::min(t1, t2);      // 取最近的
            // t = std::min(t1, t2); 
            if (t > tmin) {            // 此方法当相机位于球体内时 不会认为该球体与射线相交 // 已经修正 折射需要球体内相交
                // mormal
                Vector3f n = r_orig + t * r_dirc - center; // 计算法向量
                n.normalize();  // 法向量归一化
                h.set(t, material, n);
                return true;
            }
        }
        return false;
    }

protected:
    Vector3f center;
    float radius;

};


#endif
