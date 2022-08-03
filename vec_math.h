#pragma once
#include <math.h>

//3차원 벡터
struct vec3 {
	float x, y, z;

	vec3();

	vec3(float x, float y, float z);

	vec3 operator-() const;
	vec3 operator+(const vec3& o) const;
	vec3 operator-(const vec3& o) const;
	vec3 operator*(const float& k) const;
	vec3 operator/(const float& k) const;

	vec3& operator+=(const vec3& o);
	vec3& operator-=(const vec3& o);

	float size() const;

	vec3 normalized() const;

	void normalize();

	static float dot(const vec3& a, const vec3& b);

	static vec3 cross(const vec3& a, const vec3& b);

	static const vec3 up;
	static const vec3 right;
	static const vec3 forward;
};

//회전 처리를 위한 사원수
struct Quat {
	float w;
	vec3 v;

	Quat();
	Quat(float w, vec3 v);
	Quat(float w, float x, float y, float z);

	float sq_norm() const;
	Quat conj() const;

	Quat operator-() const;
	Quat operator*(const float& k) const;
	Quat operator/(const float& k) const;
	Quat operator*(const Quat& o) const;
	Quat operator*(const vec3& o) const;

	static Quat Euler(float x, float y, float z);
	static Quat Euler(vec3 r);

	static vec3 rotate(const vec3& v, const Quat& q);
};