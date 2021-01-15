#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include "mesh.hpp"
#include <tuple>

#define sqr(x) ((x) * (x))
#define eps 0.0025
#define INF INFINITY

class RevSurface : public Object3D {
// the curve will rotate line (x=pos.x and z=pos.z) as pivot
    BezierCurve *curve;
    Vector3f pos;
	AABB *box;

public:
    RevSurface(const Vector3f &pos, BezierCurve *curve, Material* material) : pos(pos), curve(curve), Object3D(material) {
        box = boundingBox();
    }

    ~RevSurface() override {
        if (curve) delete curve;
		if (box) delete box;
    }

    float solve_t(float yc) { // solve y(t)=yc
		// assert(0 <= yc && yc <= curve->height);
		float t = .5, ft, dft;
		for (int i = 10; i--; )
		{
			if (t < 0) t = 0;
			else if (t > 1) t = 1;
			ft = curve->getpos(t).y() - yc, dft = curve->getdir(t).y();
			if (std::abs(ft) < eps)
				return t;
			t -= ft / dft;
		}
		return -1;
	}
	virtual Vector3f change_for_bezier(Vector3f inter_p) {
		float t = solve_t(inter_p.y() - pos.y());
		float u = atan2(inter_p.z() - pos.z(), inter_p.x() - pos.x()); // between -M_PI ~ M_PI
		if (u < 0)
			u += 2 * M_PI;
		return Vector3f(u, t, 0);
	}
	float get_sphere_intersect(Ray ray, Vector3f o, float r) {
		Vector3f ro = o - ray.getOrigin();
		float b = Vector3f::dot(ray.getDirection(), ro);
		float d = sqr(b) - Vector3f::dot(ro, ro) + sqr(r);
		if (d < 0) return -1;
		else d = sqrt(d);
		float t = b - d > eps ? b - d : b + d > eps? b + d : -1;
		if (t < 0)
			return -1;
		return t;
	}

	bool intersect(const Ray &ray, Hit &h, float tmin) override {
		if (!box->intersect(ray, h, tmin)) return false;
		float final_dis = INF;
		// check for |dy|<eps
		if (std::abs(ray.getDirection().y()) < 5e-4)
		{
			float dis_to_axis = (Vector3f(pos.x(), ray.getOrigin().y(), pos.z()) - ray.getOrigin()).length();
			float hit = ray.pointAtParameter(dis_to_axis).y();
			if (hit < pos.y() + eps || hit > pos.y() + curve->height - eps)
				return false;
				// return std::make_pair(INF, Vector3f());
			// solve function pos.y()+y(t)=ray.getOrigin().y() to get x(t)
			float t = solve_t(hit - pos.y());
			if (t < 0 || t > 1)
				return false;
				//return std::make_pair(INF, Vector3f());
			Vector3f loc = curve->getpos(t);
			float ft = pos.y() + loc.y() - hit;
			if (std::abs(ft) > eps)
				return false;
				// return std::make_pair(INF, Vector3f());
			// assume sphere (pos.x(), pos.y() + loc.y(), pos.z()) - loc.x()
			final_dis = get_sphere_intersect(ray, Vector3f(pos.x(), pos.y() + loc.y(), pos.z()), loc.x());
			if (final_dis < 0)
				return false;
				// return std::make_pair(INF, Vector3f());
			Vector3f inter_p = ray.pointAtParameter(final_dis);
			// printf("y %f small!!!",std::abs((inter_p - Vector3f(pos.x(), inter_p.y(), pos.z())).squaredLength() - sqr(loc.x())));
			if (std::abs((inter_p - Vector3f(pos.x(), inter_p.y(), pos.z())).squaredLength() - sqr(loc.x())) > 1e-1)
				return false;
				// return std::make_pair(INF, Vector3f());
			// second iteration, more accuracy
			hit = inter_p.y();
			if (hit < pos.y() + eps || hit > pos.y() + curve->height - eps)
				return false;
				// return std::make_pair(INF, Vector3f());
			t = solve_t(hit - pos.y());
			loc = curve->getpos(t);
			ft = pos.y() + loc.y() - hit;
			if (std::abs(ft) > eps)
				return false;
				// return std::make_pair(INF, Vector3f());
			final_dis = get_sphere_intersect(ray, Vector3f(pos.x(), pos.y() + loc.y(), pos.z()), loc.x());
			if (final_dis < 0)
				return false;
				// return std::make_pair(INF, Vector3f());
			inter_p = ray.pointAtParameter(final_dis);
			if (std::abs((inter_p - Vector3f(pos.x(), hit, pos.z())).squaredLength() - sqr(loc.x())) > 1e-2)
				return false;
				// return std::make_pair(INF, Vector3f());
			// printf("---y %f small!!!",std::abs((inter_p - Vector3f(pos.x(), inter_p.y(), pos.z())).squaredLength() - sqr(loc.x())));
			h.set(final_dis, material, norm(inter_p));
			return true;
			// return std::make_pair(final_dis, inter_p);
		}
		// printf("y big\n");
		// check for top circle: the plane is y=pos.y() + curve->height
		// TODO
		// check for buttom circle: the plane is y=pos.y()
		// TODO
		// normal case
		// calc ay^2+by+c
		float a = 0, b = 0, c = 0, t1, t2;
		// (xo-x'+xd/yd*(y-yo))^2 -> (t1+t2*y)^2
		t1 = ray.getOrigin().x() - pos.x() - ray.getDirection().x() * ray.getOrigin().y() / ray.getDirection().y();
		t2 = ray.getDirection().x() / ray.getDirection().y();
		a += t2 * t2;
		b += 2 * t1 * t2;
		c += t1 * t1;
		// (zo-z'+zd/yd*(y-yo))^2 -> (t1+t2*y)^2
		t1 = ray.getOrigin().z() - pos.z() - ray.getDirection().z() * ray.getOrigin().y() / ray.getDirection().y();
		t2 = ray.getDirection().z() / ray.getDirection().y();
		a += sqr(t2);
		b += 2 * t1 * t2;
		c += sqr(t1);
		// ay^2+by+c -> a'(y-b')^2+c'
		c = c - b * b / 4 / a;
		b = -b / 2 / a - pos.y();
		// printf("%lf %lf %lf\n",a,b,c);
		if (0 <= b && b <= curve->height && c > curve->max2
		 || (b < 0 || b > curve->height) && std::min(sqr(b), sqr(curve->height - b)) * a + c > curve->max2) // no intersect
		 	return false;
			// return std::make_pair(INF, Vector3f());
		// float pick[20] = {0, 0, 1}; int tot = 2;
		// for (float _ = 0; _ <= 1; _ += 0.1)
		// {
		// 	float t_pick = newton2(_, a, b, c);
		// 	if (0 <= t_pick && t_pick <= 1)
		// 	{
		// 		bool flag = 1;
		// 		for (int j = 1; j <= tot; ++j)
		// 			if (std::abs(t_pick - pick[j]) < eps)
		// 				flag = 0;
		// 		if (flag)
		// 			pick[++tot] = t_pick;
		// 	}
		// }
		// std::sort(pick + 1, pick + 1 + tot);
		// for (int j = 1; j < tot; ++j)
		// 	if (getft(pick[j], a, b, c) * getft(pick[j + 1], a, b, c) <= 0)
		// 		check(pick[j], pick[j+1], (pick[j] + pick[j + 1]) * .5, ray, a, b, c, final_dis);
		for(int ind = 0; ind <= curve->num; ++ind)
		{
			// y = curve->ckpt[ind] ~ curve->ckpt[ind+1]
			// calc min(a(y-b)^2+c)
			// float lower;
			// if (curve->data[ind].y()0 <= b && b <= curve->data[ind].y()1)
			// 	lower = c;
			// else
			// 	lower = a * std::min(sqr(curve->data[ind].y()0 - b), sqr(curve->data[ind].y()1 - b)) + c;
			float t0 = curve->data[ind].t0, t1 = curve->data[ind].t1;
			// if (t0 > eps) t0 += erand48(mess) * .01;
			// if (t1 < 1 - eps) t1 -= erand48(mess) * .01;
			// if (lower <= curve->data[ind].width2)
			{
				check(t0, t1, (t0 + t1 + t0) / 3, ray, a, b, c, final_dis);
				check(t0, t1, (t1 + t0 + t1) / 3, ray, a, b, c, final_dis);
			}
		}
		if (final_dis < INF / 2) {
			h.set(final_dis, material, norm(ray.pointAtParameter(final_dis)));
			return true;
			// return std::make_pair(final_dis, ray.pointAtParameter(final_dis));
		}
		else {
			return false;
			// return std::make_pair(INF, Vector3f());
		}
	}
	bool check(float low, float upp, float init, Ray ray, float a, float b, float c, float&final_dis)
	{
		float t = newton(init, a, b, c, low, upp);
		if (t <= 0 || t >= 1)
			return false;
		Vector3f loc = curve->getpos(t);
		float x = loc.x(), y = loc.y();
		float ft = x - sqrt(a * sqr(y - b) + c);
		if (std::abs(ft) > eps)
			return false;
		// calc t for ray
		float dis = (pos.y() + y - ray.getOrigin().y()) / ray.getDirection().y();
		if (dis < eps)
			return false;
		Vector3f inter_p = ray.pointAtParameter(dis);
		if (std::abs((Vector3f(pos.x(), pos.y() + y, pos.z()) - inter_p).squaredLength() - x * x) > eps)
			return false;
		if (dis < final_dis)
		{
			final_dis = dis;
			// printf("%lf %lf %lf %lf\n",t,x , sqrt(a * sqr(y - b) + c), x - sqrt(a * sqr(y - b) + c));
			return true;
		}
		return false;
	}
	float getft(float t, float a, float b, float c)
	{
		if (t < 0) t = eps;
		if (t > 1) t = 1 - eps;
		Vector3f loc = curve->getpos(t);
		float x = loc.x(), y = loc.y();
		return x - sqrt(a * sqr(y - b) + c);
	}
	float newton(float t, float a, float b, float c, float low=eps, float upp=1-eps)
	{
		// solve sqrt(a(y(t)+pos.y()-b)^2+c)=x(t)
		// f(t) = x(t) - sqrt(a(y(t)+pos.y()-b)^2+c)
		// f'(t) = x'(t) - a(y(t)+pos.y()-b)*y'(t) / sqrt(...)
		// if t is not in [0, 1] then assume f(t) is a linear function
		float ft, dft, x, y, dx, dy, sq;
		Vector3f loc, dir;
		for (int i = 10; i--; )
		{
			if (t < 0) t = low;
			if (t > 1) t = upp;
			loc = curve->getpos(t), dir = curve->getdir(t);
			x = loc.x(), dx = dir.x();
			y = loc.y(), dy = dir.y();
			// printf("%lf %lf %lf\n",t,x,y);
			sq = sqrt(a * sqr(y - b) + c);
			ft = x - sq;
			dft = dx - a * (y - b) * dy / sq;
			if (std::abs(ft) < eps)
				return t;
			t -= ft / dft;
		}
		return -1;
	}
	float newton2(float t, float a, float b, float c)
	{
		float dft, ddft, y, dx, dy, ddx, ddy, sq;
		Vector3f loc, dir, dir2;
		for (int i = 5; i--; )
		{
			if (t < 0) t = eps;
			if (t > 1) t = 1 - eps;
			loc = curve->getpos(t), dir = curve->getdir(t), dir2 = curve->getdir2(t);
			y = loc.y(), dx = dir.x(), dy = dir.y();
			ddx = dir2.x(), ddy = dir2.y();
			sq = sqrt(a * sqr(y - b) + c);
			dft = dx - a * (y - b) * dy / sq;
			ddft = ddx - a * ((y - b) * ddy + sqr(dy)) / sq + sqr(a * (y - b) * dy) / sq / sq / sq;
			if (std::abs(dft) < eps)
				return t;
			t -= dft / ddft;
		}
		return -1;
	}
	AABB* boundingBox() {
		Vector3f _min = Vector3f(pos.x() - curve->max, pos.y(), pos.z() - curve->max);
		Vector3f _max = Vector3f(pos.x() + curve->max, pos.y() + curve->height, pos.z() + curve->max);
		return new AABB(_min, _max);
	}
	Vector3f norm(Vector3f p) {
		Vector3f tmp = change_for_bezier(p);
		Vector3f dir = curve->getdir(tmp.y());
		Vector3f d_surface = Vector3f(cos(tmp.x()), dir.y() / dir.x(), sin(tmp.x()));
		Vector3f d_circ = Vector3f(-sin(tmp.x()), 0, cos(tmp.x()));
		return Vector3f::cross(d_circ, d_surface).normalized();
	}
};

#endif //REVSURFACE_HPP
