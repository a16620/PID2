#pragma once
#include <math.h>

//3Â÷¿ø º¤ÅÍ
struct vec3 {
	double x, y, z;

	vec3();

	vec3(double x, double y, double z);

	vec3 operator-() const;
	vec3 operator+(const vec3& o) const;
	vec3 operator-(const vec3& o) const;
	vec3 operator*(const double& k) const;
	vec3 operator/(const double& k) const;

	vec3& operator+=(const vec3& o);
	vec3& operator-=(const vec3& o);

	double size() const;

	vec3 normalized() const;

	void normalize();

	static double dot(const vec3& a, const vec3& b);

	static vec3 cross(const vec3& a, const vec3& b);

	static const vec3 up;
	static const vec3 right;
	static const vec3 forward;
};


struct Quat {
	double w;
	vec3 v;

	Quat();
	Quat(double w, vec3 v);
	Quat(double w, double x, double y, double z);

	double sq_norm() const;
	Quat conj() const;

	Quat operator-() const;
	Quat operator*(const double& k) const;
	Quat operator/(const double& k) const;
	Quat operator*(const Quat& o) const;
	Quat operator*(const vec3& o) const;

	static Quat Euler(double x, double y, double z);

	static vec3 rotate(const vec3& v, const Quat& q);
};