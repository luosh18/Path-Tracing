#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
#define Tepsilon (1e-20)
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		ab = b - a;
		ac = c - a;
		// normal = Vector3f::cross(ab, ac);	// normal is calculated by mesh!!!
		// normal.normalize();
		// offset = Vector3f::dot(a, normal);
	}

	// https://math.stackexchange.com/questions/1979876/ray-casting-algorithm-in-ray-triangle-intersection
	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		//normal.normalize();
		// 1. checking intersection with plane?
		float check = Vector3f::dot(ray.getDirection(), normal);
        if (abs(check) > Tepsilon) {
			float offset = Vector3f::dot(normal, vertices[0]);
            float t = (offset - Vector3f::dot(ray.getOrigin(), normal)) / check;
            if (t > tmin) {
				// hit.set(t, material, normal);
				// return true;
				// 2. checking inside tirangle? by solving tje cordinates of the hit point in 2Dspace(ab, ac)
				
				// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
				Vector3f P = ray.getOrigin() + t * ray.getDirection();
				Vector3f C;

				Vector3f edge0 = vertices[1] - vertices[0];
				Vector3f vp0 = P - vertices[0];
				C = Vector3f::cross(edge0, vp0);
				if ( Vector3f::dot(normal, C) < 0) return false;

				Vector3f edge1 = vertices[2] - vertices[1];
				Vector3f vp1 = P - vertices[1];
				C = Vector3f::cross(edge1, vp1);
				if ( Vector3f::dot(normal, C) < 0) return false;

				Vector3f edge2 = vertices[0] - vertices[2];
				Vector3f vp2 = P - vertices[2];
				C = Vector3f::cross(edge2, vp2);
				if ( Vector3f::dot(normal, C) < 0) return false;

				hit.set(t, material, normal);
				return true;

				// float delta = Matrix2f::determinant2x2(ab.x(), ac.x(), 
				// 									   ab.y(), ac.y());							   
				// if (delta != 0.0) {
				// 	Vector3f pt = (ray.getOrigin() + t * ray.getDirection()) - vertices[0];	// 相对于a的坐标
				// 	float delta_x = Matrix2f::determinant2x2(pt.x(), ac.x(), 
				// 									   	   	 pt.y(), ac.y());
				// 	float delta_y = Matrix2f::determinant2x2(ab.x(), pt.x(), 
				// 									   		 ab.y(), pt.y());
				// 	float cord_x = delta_x / delta;		// 2Dspace(ab, ac)
				// 	float cord_y = delta_y / delta;
				// 	if ((cord_x >= 0.0) && (cord_y >= 0.0) && ((cord_x + cord_y) <= 1.0)) {	// inside triangle
				// 		hit.set(t, material, normal);
				// 		return true;
				// 	}
				// }

				// https://www.youtube.com/watch?v=HYAgJN3x4GA
				// Vector3f pt = ray.getOrigin() + t * ray.getDirection() - vertices[0];
				// Vector3f P = ray.getOrigin() + t * ray.getDirection();
				// float s1 = ac.y();
				// float s2 = ac.x();
				// float s3 = ab.y();
				// float s4 = pt.y();
				// float w1 = (vertices[0].x() * s1 + s4 * s2 - P.x() * s1) / (s3 * s2 - (ab.x()) * s1);
				// float w2 = (s4- w1 * s3) / s1;
				// if (w1 >= 0 && w2 >= 0 && (w1 + w2) <= 1) {
				// 	hit.set(t, material, normal);
				// 	return true;
				// }
            }
        }
        return false;
	}

	Vector3f normal;
	Vector3f vertices[3];
protected:
	Vector3f ab;
	Vector3f ac;
};

#endif //TRIANGLE_H
