#pragma once
#include <math.h>

#define WINDOWS

constexpr float MATH_PI = 3.14159265358979323f;

inline float math_sign(float x) noexcept {
	return signbit(x) ? -1 : 1;
}

inline bool comp_nsign(float X, float Y) noexcept {
	return signbit(X) ^ signbit(Y);
}

inline bool comp_sign(float X, float Y) noexcept {
	return !comp_nsign(X, Y);
}

inline float math_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float math_map2(float x, float in_hrange, float out_hrange) {
	return (x + in_hrange) * (in_hrange * 2) / (in_hrange*2) + out_hrange;
}

inline float math_constrain(float x, float min, float max) {
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

inline float GetTimeDelta(Time t1, Time t2) {
	auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	return static_cast<float>(delta) / 1000;
}

#else
#include <Arduino.h>
using Time = unsigned long;

#define now millis

inline float GetTimeDelta(Time t1, Time t2) {
	auto delta = t2 - t1;
	return static_cast<float>(delta) / 1000;
}

#endif
