#pragma once
#define WINDOWS

#ifdef WINDOWS
#include "arduino.h"
#include <chrono>

using Time = std::chrono::system_clock::time_point;

inline Time now() {
	return std::chrono::system_clock::now();
}

inline double GetTimeDelta(Time t1, Time t2) {
	auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
	return static_cast<double>(delta) / 1000;
}

#else

using Time = unsigned long;

#define now millis

inline double GetTimeDelta(Time t1, Time t2) {
	auto delta = t2 - t1;
	return static_cast<double>(delta) / 1000;
}

#endif
