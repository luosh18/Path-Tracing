#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"
#include "group.hpp"

class Light {
public:
    Light() = default;

    virtual ~Light() = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;
    virtual Vector3f PTIllumiination(const Ray &ray, Group *group) = 0;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    Vector3f PTIllumiination(const Ray &ray, Group *group) {
        Vector3f toLight = -direction;
        Hit hit;
        if (group->intersect(Ray(ray.getOrigin(), toLight), hit, 0)) {
            return Vector3f::ZERO;
        }
        else {
            float cos_theta = Vector3f::dot(toLight, ray.getDirection());
            cos_theta = cos_theta > 0 ? cos_theta : 0;
            return color * cos_theta;
        }
    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    Vector3f PTIllumiination(const Ray &ray, Group *group) {
        Vector3f toLight = position - ray.getOrigin();
        Hit hit;
        if (group->intersect(Ray(ray.getOrigin(), toLight), hit, 0)) {
            return Vector3f::ZERO;
        }
        else {
            float cos_theta = Vector3f::dot(toLight, ray.getDirection());
            cos_theta = cos_theta > 0 ? cos_theta : 0;
            return color * cos_theta;
        }
    }

private:

    Vector3f position;
    Vector3f color;

};

#endif // LIGHT_H
