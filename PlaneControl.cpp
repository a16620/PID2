#include "PlaneControl.h"

PlaneGyro PlaneGyro::inst;

YawController::YawController()
{
	SetOutputLimit(100);
}

void YawController::Set(const double& angle)
{
	double v = GetControlValue(angle, PlaneGyro::getInstance().rot_vec.y);
	short cv = math_map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetRudder(cv);
}

PitchController::PitchController()
{
	SetOutputLimit(100);
}

void PitchController::Set(const double& angle)
{
	double v = GetControlValue(angle, PlaneGyro::getInstance().rot_vec.x);
	short cv = math_map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetElev(cv);
}

RollController::RollController()
{
	SetOutputLimit(100);
}

void RollController::Set(const double& angle)
{
	double v = GetControlValue(angle, PlaneGyro::getInstance().rot_vec.z);
	short cv = math_map(v, -100, 100, -MATH_PI / 2, MATH_PI / 2);
	PlaneController::SetAiler1(cv);
	PlaneController::SetAiler2(-cv);
}

void PlaneGyro::update()
{
	//읽기
	vec3 sensor_input;

	rot_vec = sensor_input - calibration_rotation;

	//쿼터니언 변환
	rotation = Quat::Euler(rot_vec.x, rot_vec.y, rot_vec.z);
}

void PlaneGyro::calibrate()
{
	vec3 sensor_input;

	calibration_rotation = sensor_input;
	update();
}

TimeChecker TimeChecker::inst;

TimeChecker::TimeChecker()
{
	last_checked = now();
	delta = 0;
}

void TimeChecker::update()
{
	auto _now = now();
	delta = GetTimeDelta(last_checked, _now);
	last_checked = _now;
}

double TimeChecker::deltaTime()
{
	return delta;
}

TimeChecker& TimeChecker::getInstance()
{
	return inst;
}
