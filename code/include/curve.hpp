#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>

#include <algorithm>
#define C_Eps 1e-6

class Curve : public Object3D {
protected:
    std::vector<Vector2f> controls;
public:
    explicit Curve(std::vector<Vector2f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        return false;
    }

    std::vector<Vector2f> &getControls() {
        return controls;
    }

    //virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;
};

class BezierCurve : public Curve {
public:
    struct D {
        float t0, t1, width, y0, y1, width2;
    }data[70];

    explicit BezierCurve(const std::vector<Vector2f> &points, float r = 0.365): 
            Curve(points), r(r), p(controls) 
    {
        num = controls.size();
        n = controls.size() - 1;
        d.resize(controls.size());

        for (int i=0; i<=n; i++) {
            d[i] = p[0];
            for (int j=0; j<=n-i; j++) {
                p[j] = p[j+1] - p[j];
            }
        }
        float n_down = 1, fac = 1, nxt = n;
        for (int i=0; i<=n; i++, nxt--) {
            fac = fac * (i==0 ? 1 : i);
            d[i] = d[i] * n_down / fac;
            n_down *= nxt;
        }
        max = 0;
        float interval = 1.0 / (num - 1), c = 0;
        for (int cnt = 0; cnt <= num; c += interval, cnt++) {
            data[cnt].width = 0;
			data[cnt].t0 = std::max(0.f, c - r);
			data[cnt].t1 = std::min(1.f, c + r);
			data[cnt].y0 = getpos(data[cnt].t0).y();
			data[cnt].y1 = getpos(data[cnt].t1).y();
			for (float t = data[cnt].t0; t <= data[cnt].t1; t += 0.00001)
			{
				Vector3f pos = getpos(t);
				if (data[cnt].width < pos.x())
					data[cnt].width = pos.x();
			}
			if (max < data[cnt].width)
				max = data[cnt].width;
			data[cnt].width += C_Eps;
			data[cnt].width2 = data[cnt].width * data[cnt].width;
        }
        max += C_Eps;
        max2 = max * max;
        height = getpos(1).y();
    }

    Vector3f getpos(float t) {
        Vector2f ans(0.f);
        float t_pow = 1;
        for (int i=0; i<=n; i++) {
            ans += d[i] * t_pow;
            t_pow *= t;
        }
        return Vector3f(ans, 0.f);
    }

    Vector3f getdir(float t) {
        Vector2f ans(0.f);
        float t_pow = 1;
		for(int i=1; i<=n; i++) {
			ans += d[i] * i * t_pow;
			t_pow *= t;
		}
		return Vector3f(ans, 0.f);
    }

    Vector3f getdir2(float t) {
        Vector2f ans(0.f);
		float t_pow = 1;
		for(int i=2; i<=n; i++) {
			ans += d[i] * i * (i - 1) * t_pow;
			t_pow *= t;
		}
		return Vector3f(ans, 0.f);
	}

    std::vector<Vector2f> d, p;
    int n;
    float max, height, max2, r, num;
};

#endif // CURVE_HPP
