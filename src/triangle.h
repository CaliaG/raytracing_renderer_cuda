#pragma once

#include "hitable_object.h"
#include "material.h"

class triangle : public hitable_object {
public:
    __device__ triangle() {};
    __device__ triangle(vec3 _a, vec3 _b, vec3 _c, const material* mat)
        : A(_a), B(_b), C(_c), _m(mat) {
            normal = unit_vector(cross(B - A, C - A));
        };

    __device__ virtual bool hit(const ray& r, float tmin, float tmax, hit_record& hrec) override;
    __device__ virtual bool bounding_box(float t0, float t1, AABB& box) const override;
    
    __device__ virtual ~triangle() noexcept override;

    __device__ virtual object_type get_object_type() const override {
        return object_type::TRIANGLE;
    }

private:
	vec3 A;
	vec3 B;
	vec3 C;
	vec3 normal;
    const material* _m = nullptr;
};

__device__ 
bool triangle::hit(const ray& r, float tmin, float tmax, hit_record& hrec) {
    const float EPSILON = .0000001f;
	vec3 edge1, edge2, h, s, q;
	float a, f, u, v;

	edge1 = B - A;
	edge2 = C - A;
	h = cross(r.direction, edge2);
	a = dot(edge1, h);

	if (a > -EPSILON && a < EPSILON)
		return false;  // Parallel to the triangle

	f = 1.0 / a;
	s = r.origin - A;
	u = f * dot(s, h);

	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);
	v = f * dot(r.direction, q);

	if (v < 0.0 || u + v > 1.0)
		return false;

	float t = f * dot(edge2, q);

	if (t > t_min && t < t_max) {
		rec.t = t;
		rec.p = r.point_at_parameter(t);
		rec.normal = normal;
		return true;
	} else
		return false;
}

__device__
bool sphere::bounding_box(float t0, float t1, AABB& box) const {
    float sx = A.x() < B.x() ? A.x() : B.x();
    sx = C.x() < sx ? C.x() : sx;
    float sy = A.y() < B.y() ? A.y() : B.y();
    sy = C.y() < sy ? C.y() : sy;
    float sz = A.z() < B.z() ? A.z() : B.z();
    sz = C.z() < sz ? C.z() : sz;

    float lx = A.x() > B.x() ? A.x() : B.x();
    lx = C.x() > lx ? C.x() : lx;
    float ly = A.y() > B.y() ? A.y() : B.y();
    ly = C.y() > ly ? C.y() : ly;
    float lz = A.z() > B.z() ? A.z() : B.z();
    lz = C.z() > lz ? C.z() : lz;

    box = AABB(vec3(sx,sy,sz), vec3(lx,ly,lz));
    return true;
}

__device__
triangle::~triangle() noexcept {
    //printf("Deleting triangle object at %p\n", this);
    if (_m) {
        //printf("--Deleting material object at %p\n", _m);
        delete _m;
    }
}
