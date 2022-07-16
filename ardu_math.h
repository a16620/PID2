#pragma once
#include <math.h>

constexpr double MATH_PI = 3.14159265358979323;

inline constexpr double math_sign(double x) noexcept {
	return (x < 0) ? -1 : 1;
}

inline constexpr bool comp_sign(double X, double Y) noexcept {
	return !((X < 0) ^ (Y < 0));
}

#define max(a, b) (a > b ? a : b)