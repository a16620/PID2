#pragma once
#include <math.h>
#include "arduino.h"


//3Â÷¿ø º¤ÅÍ
struct vec3 {
	double x, y, z;

	vec3() {
		x = y = z = 0;
	}

	vec3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	vec3 operator-() const {
		return vec3(-x, -y, -z);
	}

	vec3 operator+(const vec3& o) const {
		return vec3(x + o.x, y + o.y, z + o.z);
	}

	vec3 operator-(const vec3& o) const {
		return vec3(x - o.x, y - o.y, z - o.z);
	}

	vec3 operator*(const double& k) const {
		return vec3(k * x, k * y, k * z);
	}

	vec3 operator/(const double& k) const {
		return vec3(k / x, k / y, k / z);
	}

	vec3& operator+=(const vec3& o) {
		x += o.x;
		y += o.y;
		z += o.z;

		return *this;
	}

	vec3& operator-=(const vec3& o) {
		x -= o.x;
		y -= o.y;
		z -= o.z;

		return *this;
	}

	double size() const {
		return sqrt(sq(x) + sq(y) + sq(z));
	}

	vec3 normalized() const {
		return *this / this->size();
	}

	void normalize() {
		const auto size = this->size();
		x /= size;
		y /= size;
		z /= size;
	}

	static double dot(const vec3& a, const vec3& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static vec3 cross(const vec3& a, const vec3& b) {
		vec3 r;

		r.x = a.y * b.z - b.y * a.z;
		r.y = a.z * b.x - b.z * a.x;
		r.z = a.x * b.y - b.x * a.y;

		return r;
	}

	static const vec3 up;
	static const vec3 right;
	static const vec3 forward;
};

const vec3 vec3::up = vec3(0, 1, 0);
const vec3 vec3::right = vec3(1, 0, 0);
const vec3 vec3::forward = vec3(0, 0, 1);

struct Quat {
	double w;
	vec3 v;

	Quat() {
		w = 0;
		v = vec3();
	}

	Quat(double w, vec3 v) {
		this->w = w;
		this->v = v;
	}

	Quat(double w, double x, double y, double z) {
		this->w = w;
		v = vec3(x, y, z);
	}

	static Quat Euler(double x, double y, double z) {
		x /= 2;
		y /= 2;
		z /= 2;

		Quat q;

		const double sin_x = sin(x), sin_y = sin(y), sin_z = sin(z),
			cos_x = cos(x), cos_y = cos(y), cos_z = cos(z);

		q.w = cos_z * cos_y * cos_x + sin_z * sin_y * sin_x;
		q.v.x = cos_z * cos_y * sin_x - sin_z * sin_y * cos_x;
		q.v.y = cos_z * sin_y * cos_x + sin_z * cos_y * sin_x;
		q.v.z = sin_z * cos_y * cos_x - cos_z * sin_y * sin_x;

		return q;
	}

	double sq_norm() const {
		return sq(w) + sq(v.x) + sq(v.y) + sq(v.z);
	}

	Quat conj() const {
		return Quat(w, -v);
	}

	Quat operator-() const {
		return Quat(-w, -v);
	}

	Quat operator*(const double& k) const {
		return Quat(w * k, v * k);
	}

	Quat operator/(const double& k) const {
		return Quat(w / k, v / k);
	}

	Quat operator*(const Quat& o) const {
		Quat r;

		r.w = w * o.w - vec3::dot(v, o.v);
		r.v = v * o.w + o.v * w + vec3::cross(v, o.v);

		return r;
	}

	Quat operator*(const vec3& o) const {
		Quat r;

		r.w = -vec3::dot(v, o);
		r.v = o * w + vec3::cross(v, o);

		return r;
	}

	static vec3 rotate(const vec3& v, const Quat& q) {
		return (q * v * q.conj()).v;
	}
};