#pragma once
#include <math.h>

#define WINDOWS

constexpr double MATH_PI = 3.14159265358979323;

inline double math_sign(double x) noexcept {
	return (x < 0) ? -1 : 1;
}

inline bool comp_sign(double X, double Y) noexcept {
	return !((X < 0) ^ (Y < 0));
}

inline double math_map(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline double math_map2(double x, double in_hrange, double out_hrange) {
	return (x + in_hrange) * (in_hrange * 2) / (in_hrange*2) + out_hrange;
}

inline double math_constrain(double x, double min, double max) {
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

#ifdef WINDOWS
#include <chrono>

#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)
#define sq(x) (pow(x, 2))

unsigned long millis();
long map(long x, long in_min, long in_max, long out_min, long out_max);


using Time = std::chrono::system_clock::time_point;

inline Time now() {
	return std::chrono::system_clock::now();
}

inline double GetTimeDelta(Time t1, Time t2) {
	auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	return static_cast<double>(delta) / 1000;
}

#else
#include <Arduino.h>
using Time = unsigned long;

#define now millis

inline double GetTimeDelta(Time t1, Time t2) {
	auto delta = t2 - t1;
	return static_cast<double>(delta) / 1000;
}

#endif
