#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>
#include <algorithm>    // std::max
#include <random>

class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction).normalized();
        //this->up = up.normalized();
        //this->horizontal = Vector3f::cross(this->direction, this->up);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle, float _aperture, float _focus) : 
            Camera(center, direction, up, imgW, imgH) {
        // angle is in radian.
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays
        halfAperture = _aperture / 2.0;
        focus = _focus;
    	tanHalfAngle = tan(angle / 2.0);
		imageAspectRatio = float(imgW) / float(imgH);	// assuming width > height 
		rotation = Matrix3f(this->horizontal, -this->up, this->direction);	// 相机的旋转矩阵 
    }

    Ray generateRay(const Vector2f &point) override {
        // 
		float Px = (2 * ((point.x() + 0.5) / width) - 1) * tanHalfAngle * imageAspectRatio; 
		float Py = (1 - 2 * ((point.y() + 0.5) / height)) * tanHalfAngle;
        Vector3f focusPoint(Px*focus, Py*focus, focus);
        Vector3f paa = randomPointAtAperture();
        Vector3f direction = (focusPoint-paa);
		return Ray(center + (rotation * paa), (rotation * direction).normalized());
    }

protected:
    Vector3f randomPointAtAperture() {      // genarate a point on aperture disk
        if (halfAperture == 0.f) return Vector3f::ZERO;
        float theta = 2 * M_PI * distribution(genarator);
        return Vector3f(halfAperture*cos(theta), halfAperture*sin(theta), 0);
    }

    float tanHalfAngle;			// 视角 tan(alpha/2)
	float imageAspectRatio;	// 宽高比
    Matrix3f rotation;		// 相机的旋转矩阵

    float halfAperture; // 光圈
    float focus;        // 焦距
    std::default_random_engine genarator;
    std::uniform_real_distribution<float> distribution = std::uniform_real_distribution<float>(0, 1); 
};

#endif //CAMERA_H
