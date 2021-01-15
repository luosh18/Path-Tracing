#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:
    // TODO: add emission and config color, shading
    explicit Material(const Vector3f &d_color, const float &d, 
                      const Vector3f &s_color, const float &s, 
                      const Vector3f &r_color, const float &ri, const float& r, 
                      const Vector3f &e): 
                      diffuseColor(d_color), diffuse(d), 
                      specularColor(s_color), specular(s), 
                      refractColor(r_color), refractiveIndex(ri), refract(r), 
                      emission(e) {}

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    virtual float getDiffuse() const {
        return diffuse;
    }

    virtual Vector3f getSpecularColor() const {
        return specularColor;
    }

    // virtual float getShininess() const {
    //     return shininess;
    // }

    virtual float getSpecular() const {
        return specular;
    }

    virtual Vector3f getRefractColor() const {
        return refractColor;
    }

    virtual float getRefractiveIndex() const {
        return refractiveIndex;
    }

    virtual float getRefract() const {
        return refract;
    }

    virtual Vector3f getEmission() const {
        return emission;
    }

    // phong shading // in path tracing phong shading is 木大
    // Vector3f Shade(const Ray &ray, const Hit &hit,
    //                const Vector3f &dirToLight, 
    //                const Vector3f &lightColor) {
    //     Vector3f shaded = Vector3f::ZERO;
    //     // 
    //     Vector3f N = hit.getNormal().normalized();
    //     Vector3f L = dirToLight.normalized();
    //     Vector3f V = - (ray.getDirection().normalized());
    //     Vector3f R = 2.0 * (Vector3f::dot(L, N)) * N - L;
    //     shaded = lightColor * (diffuseColor * (ReLU(Vector3f::dot(L, N))) + specularColor * pow(ReLU(Vector3f::dot(V, R)), shininess));;
    //     return shaded;
    // }

protected:
    Vector3f diffuseColor;
    float diffuse;          // ratio of diffuse
    Vector3f specularColor;
    // float shininess;
    float specular;         // ratio of specular
    Vector3f refractColor;
    float refractiveIndex;
    float refract;          // ratio of refract
    Vector3f emission;      // including color and brightness

    // suggest that diffuse+specular+refract = 1

    float ReLU(const float &x) {
        if (x > 0.0) return x;
        else return 0.0;
        // if (x > 0.0) return x;
        // else return -x;
    }
};


#endif // MATERIAL_H
