#pragma once
#include "PIDControl.h"

class PlaneController {
public:
	static void SetAiler1(const double& angle);
	static void SetAiler2(const double& angle);

	static void SetRudder(const double& angle);
	
	static void SetElev(const double& angle);

	//!!ºÐ¸®
	static double YawAngle();
	static double RollAngle();
	static double PitchAngle();
};

class YawController : public PIDControl {
public:
	YawController() {
		SetOutputLimit(100);
		Begin();
	}

	void Set(const double& angle) {
		double error = angle - PlaneController::YawAngle();
		double v = GetControlValue(error);
		short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI/2);
		PlaneController::SetRudder(cv);
	}
};

class PitchController : public PIDControl {
	PitchController() {
		SetOutputLimit(100);
		Begin();
	}

	void Set(const double& angle) {
		double error = angle - PlaneController::PitchAngle();
		double v = GetControlValue(error);
		short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
		PlaneController::SetElev(cv);
	}
};

class RollController : public PIDControl {
public:
	RollController() {
		SetOutputLimit(100);
		Begin();
	}

	void Set(const double& angle) {
		double error = angle - PlaneController::RollAngle();
		double v = GetControlValue(error);
		short cv = map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
		PlaneController::SetAiler1(cv);
		PlaneController::SetAiler2(-cv);
	}
};